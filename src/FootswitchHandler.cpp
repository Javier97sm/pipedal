// Copyright (c) 2026 Robin Davies
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "pch.h"
#include "FootswitchHandler.hpp"
#include "Lv2Log.hpp"
#include "ss.hpp"

#include <cstring>
#include <cerrno>
#include <vector>
#include <algorithm>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <linux/spi/spidev.h>

using namespace pipedal;

namespace
{
    constexpr const char *INPUT_DIR = "/dev/input";

    // Map a kernel key code to a physical footswitch index (0..3, = FS1..FS4).
    // The device tree maps each footswitch to KEY_F1..KEY_F4 in panel order, so
    // the mapping is direct. Index order keeps the chord pairs aligned:
    // (0,1)=FS1+FS2 -> previous bank, (2,3)=FS3+FS4 -> next bank.
    int switchIndexForKeyCode(int code)
    {
        switch (code)
        {
        case KEY_F1: return 0; // FS1
        case KEY_F2: return 1; // FS2
        case KEY_F3: return 2; // FS3
        case KEY_F4: return 3; // FS4
        default: return -1;
        }
    }

    bool deviceHasKey(int fd, int code)
    {
        unsigned long bits[(KEY_MAX + 1) / (8 * sizeof(unsigned long)) + 1];
        std::memset(bits, 0, sizeof(bits));
        if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0)
            return false;
        return (bits[code / (8 * sizeof(unsigned long))] >> (code % (8 * sizeof(unsigned long)))) & 1UL;
    }
}

// ---------------------------------------------------------------------------
// SpiLedStrip (SK9822)
// ---------------------------------------------------------------------------

SpiLedStrip::SpiLedStrip(int ledCount)
    : ledCount_(ledCount > 0 ? ledCount : 0)
{
}

SpiLedStrip::~SpiLedStrip()
{
    Close();
}

bool SpiLedStrip::Open(const std::string &device)
{
    std::lock_guard<std::mutex> guard(mutex_);
    if (fd_ >= 0)
        return true;

    // Write-only: the LEDs need no MISO, and O_WRONLY avoids needing read perms.
    int fd = open(device.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0)
    {
        Lv2Log::info(SS("SpiLedStrip: no LED device " << device
            << " (" << strerror(errno) << "). Snapshot LEDs disabled."));
        return false;
    }

    uint8_t mode = SPI_MODE_0; // SK9822 = CPOL0/CPHA0
    uint8_t bits = 8;
    uint32_t hz = 8000000; // 8 MHz is safe for a short strip
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0)
    {
        Lv2Log::warning(SS("SpiLedStrip: failed to configure " << device
            << " (" << strerror(errno) << "). Snapshot LEDs disabled."));
        close(fd);
        return false;
    }

    fd_ = fd;
    lastIndex_ = -2; // force the next ShowSingleWhite() to actually write.
    Lv2Log::info(SS("SpiLedStrip: using LED device " << device
        << " (" << ledCount_ << " LEDs)."));
    return true;
}

void SpiLedStrip::Close()
{
    std::lock_guard<std::mutex> guard(mutex_);
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

void SpiLedStrip::ShowSingleWhite(int index)
{
    std::lock_guard<std::mutex> guard(mutex_);
    if (fd_ < 0 || index == lastIndex_)
        return;
    lastIndex_ = index;

    std::vector<std::array<uint8_t, 3>> rgb(ledCount_, std::array<uint8_t, 3>{0, 0, 0});
    if (index >= 0 && index < ledCount_)
        rgb[index] = { 255, 255, 255 }; // white

    Write(rgb);
}

void SpiLedStrip::Write(const std::vector<std::array<uint8_t, 3>> &rgb)
{
    // Caller holds mutex_ and has checked fd_ >= 0.
    // Frame: 4-byte start (zeros) + per-LED [0xE0|bright, B, G, R] + 4-byte end.
    std::vector<uint8_t> buf;
    buf.reserve(4 + ledCount_ * 4 + 4);

    buf.insert(buf.end(), { 0x00, 0x00, 0x00, 0x00 }); // start frame
    for (int i = 0; i < ledCount_; ++i)
    {
        buf.push_back(0xE0 | (BRIGHTNESS & 0x1F));
        buf.push_back(rgb[i][2]); // B
        buf.push_back(rgb[i][1]); // G
        buf.push_back(rgb[i][0]); // R
    }
    buf.insert(buf.end(), { 0x00, 0x00, 0x00, 0x00 }); // end frame (SK9822)

    ssize_t n = write(fd_, buf.data(), buf.size());
    if (n != (ssize_t)buf.size())
    {
        Lv2Log::warning(SS("SpiLedStrip: short/failed write (" << strerror(errno) << ")."));
    }
}

// ---------------------------------------------------------------------------
// FootswitchHandler
// ---------------------------------------------------------------------------

FootswitchHandler::FootswitchHandler(
    Callbacks callbacks,
    const std::string &deviceName)
    : callbacks_(std::move(callbacks)),
      deviceName_(deviceName)
{
    ResetState();
}

FootswitchHandler::~FootswitchHandler()
{
    Stop();
}

void FootswitchHandler::ResetState()
{
    for (int i = 0; i < SWITCH_COUNT; ++i)
    {
        state_[i] = SwitchState::Released;
        pressTime_[i] = clock::time_point{};
    }
}

void FootswitchHandler::Start()
{
    if (thread_.joinable())
        return;

    // Snapshot-indicator LEDs are independent of the input device; bring them
    // up here and start dark.
    if (ledStrip_.Open())
        ledStrip_.Clear();

    wakeEventFd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (wakeEventFd_ < 0)
    {
        Lv2Log::error(SS("FootswitchHandler: failed to create eventfd. " << strerror(errno)));
        return;
    }
    stop_ = false;
    thread_ = std::thread([this]() { ThreadProc(); });
}

void FootswitchHandler::Stop()
{
    if (!thread_.joinable())
    {
        if (wakeEventFd_ >= 0)
        {
            close(wakeEventFd_);
            wakeEventFd_ = -1;
        }
        return;
    }
    stop_ = true;
    // Wake the poll() in the worker thread.
    uint64_t one = 1;
    (void)!write(wakeEventFd_, &one, sizeof(one));
    thread_.join();

    close(wakeEventFd_);
    wakeEventFd_ = -1;

    ledStrip_.Clear();
    ledStrip_.Close();
}

void FootswitchHandler::SetSelectedSnapshot(int snapshotIndex)
{
    // ShowSingleWhite() is thread-safe and suppresses redundant updates.
    ledStrip_.ShowSingleWhite(snapshotIndex);
}

int FootswitchHandler::OpenDevice()
{
    DIR *dir = opendir(INPUT_DIR);
    if (!dir)
        return -1;

    int foundFd = -1;
    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        if (std::strncmp(entry->d_name, "event", 5) != 0)
            continue;

        std::string path = std::string(INPUT_DIR) + "/" + entry->d_name;
        int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
            continue;

        char name[256] = {0};
        if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) < 0 ||
            deviceName_ != name ||
            !deviceHasKey(fd, KEY_F1))
        {
            close(fd);
            continue;
        }

        // Claim the device exclusively so the footswitches are not also
        // delivered to the console as F1..F4 keystrokes. Non-fatal if it fails.
        if (ioctl(fd, EVIOCGRAB, 1) < 0)
        {
            Lv2Log::warning(SS("FootswitchHandler: could not grab '" << path
                << "' (" << strerror(errno) << "). Continuing without exclusive access."));
        }

        Lv2Log::info(SS("FootswitchHandler: using input device " << path
            << " (\"" << name << "\")."));
        foundFd = fd;
        break;
    }
    closedir(dir);
    return foundFd;
}

void FootswitchHandler::HandleKey(int switchIndex, bool pressed, clock::time_point now)
{
    if (switchIndex < 0 || switchIndex >= SWITCH_COUNT)
        return;

    // Pairs are (0,1) -> previous preset and (2,3) -> next preset.
    const int partner = switchIndex ^ 1;

    if (pressed)
    {
        // Ignore repeats / duplicate down events.
        if (state_[switchIndex] != SwitchState::Released)
            return;

        if (state_[partner] == SwitchState::Down)
        {
            // The partner is still physically held: this is a chord. Mark both
            // consumed so their later releases do not fire snapshots and any
            // pending long-press is abandoned.
            state_[switchIndex] = SwitchState::Consumed;
            state_[partner] = SwitchState::Consumed;
            if (switchIndex < 2)
            {
                Lv2Log::info("FootswitchHandler: chord -> previous preset.");
                if (callbacks_.onPreviousPreset)
                    callbacks_.onPreviousPreset();
            }
            else
            {
                Lv2Log::info("FootswitchHandler: chord -> next preset.");
                if (callbacks_.onNextPreset)
                    callbacks_.onNextPreset();
            }
        }
        else
        {
            state_[switchIndex] = SwitchState::Down;
            pressTime_[switchIndex] = now; // start the long-press clock (FS1/FS4).
        }
    }
    else // released
    {
        if (state_[switchIndex] == SwitchState::Down)
        {
            // Held and released on its own (a tap), never overlapping the
            // partner and not held long enough for a long-press: commit it as a
            // snapshot change.
            Lv2Log::info(SS("FootswitchHandler: switch " << (switchIndex + 1)
                << " -> snapshot " << (switchIndex + 1) << "."));
            if (callbacks_.onSnapshot)
                callbacks_.onSnapshot(switchIndex);
        }
        state_[switchIndex] = SwitchState::Released;
    }
}

void FootswitchHandler::FireExpiredHolds(clock::time_point now)
{
    // Only the two outer switches have a long-press action: FS1 -> previous
    // bank, FS4 -> next bank. A switch still Down past the hold window (and not
    // claimed by a chord) commits its bank step and is marked Consumed so the
    // eventual release does not also fire a snapshot.
    if (state_[0] == SwitchState::Down && (now - pressTime_[0]) >= HOLD_WINDOW)
    {
        Lv2Log::info("FootswitchHandler: switch 1 long-press -> previous bank.");
        state_[0] = SwitchState::Consumed;
        if (callbacks_.onPreviousBank)
            callbacks_.onPreviousBank();
    }
    if (state_[3] == SwitchState::Down && (now - pressTime_[3]) >= HOLD_WINDOW)
    {
        Lv2Log::info("FootswitchHandler: switch 4 long-press -> next bank.");
        state_[3] = SwitchState::Consumed;
        if (callbacks_.onNextBank)
            callbacks_.onNextBank();
    }
}

int FootswitchHandler::NextTimeoutMs(clock::time_point now) const
{
    // Wake when the soonest pending long-press is due; block indefinitely if
    // neither outer switch (FS1/FS4) is held.
    int timeout = -1;
    const int outer[2] = { 0, 3 };
    for (int i : outer)
    {
        if (state_[i] == SwitchState::Down)
        {
            auto deadline = pressTime_[i] + HOLD_WINDOW;
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
            if (remaining < 0)
                remaining = 0;
            if (timeout < 0 || remaining < timeout)
                timeout = (int)remaining;
        }
    }
    return timeout;
}

void FootswitchHandler::ThreadProc()
{
    bool loggedWaiting = false;

    while (!stop_)
    {
        int fd = OpenDevice();
        if (fd < 0)
        {
            if (!loggedWaiting)
            {
                Lv2Log::info(SS("FootswitchHandler: no \"" << deviceName_
                    << "\" input device found. Footswitches disabled (will keep watching)."));
                loggedWaiting = true;
            }
            // Idle, but wake immediately on Stop(). Re-scan every 5s.
            struct pollfd pfd { wakeEventFd_, POLLIN, 0 };
            poll(&pfd, 1, 5000);
            continue;
        }
        loggedWaiting = false;
        ResetState();

        while (!stop_)
        {
            struct pollfd pfds[2] = {
                { fd, POLLIN, 0 },
                { wakeEventFd_, POLLIN, 0 },
            };
            // Chords are detected by overlap (no timing), but a held outer
            // switch must wake us when its long-press window elapses.
            int n = poll(pfds, 2, NextTimeoutMs(clock::now()));

            if (stop_)
                break;

            if (n < 0)
            {
                if (errno == EINTR)
                    continue;
                Lv2Log::error(SS("FootswitchHandler: poll failed. " << strerror(errno)));
                break;
            }

            if (pfds[1].revents & POLLIN)
            {
                // Stop signal; drain the eventfd and let the loop re-check stop_.
                uint64_t v;
                (void)!read(wakeEventFd_, &v, sizeof(v));
            }

            if (pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                Lv2Log::warning("FootswitchHandler: input device disconnected.");
                break; // re-discover the device.
            }

            if (pfds[0].revents & POLLIN)
            {
                struct input_event events[64];
                for (;;)
                {
                    ssize_t bytes = read(fd, events, sizeof(events));
                    if (bytes < 0)
                    {
                        if (errno == EAGAIN || errno == EWOULDBLOCK)
                            break;
                        if (errno == EINTR)
                            continue;
                        Lv2Log::warning(SS("FootswitchHandler: read error. " << strerror(errno)));
                        bytes = 0;
                        break;
                    }
                    if (bytes == 0)
                        break;

                    size_t count = (size_t)bytes / sizeof(input_event);
                    for (size_t i = 0; i < count; ++i)
                    {
                        const input_event &ev = events[i];
                        if (ev.type != EV_KEY)
                            continue;
                        if (ev.value != 0 && ev.value != 1) // ignore autorepeat (2)
                            continue;
                        int sw = switchIndexForKeyCode(ev.code);
                        if (sw >= 0)
                            HandleKey(sw, ev.value == 1, clock::now()); // active-low keys: 1 = pressed
                    }
                }
            }

            // Commit any outer-switch long-press whose window has elapsed
            // (covers both the poll timeout and post-event re-checks).
            FireExpiredHolds(clock::now());
        }

        // EVIOCGRAB is released automatically on close.
        close(fd);
    }
}
