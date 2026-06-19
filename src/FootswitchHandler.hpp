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

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace pipedal
{
    // Minimal write-only driver for a short series string of SK9822 (APA102-
    // compatible) RGB LEDs on a spidev device. Used to indicate the currently
    // selected snapshot on the four front-panel footswitch LEDs (left to
    // right), but it has no dependency on the footswitches themselves.
    //
    // All public methods are thread-safe; SetSelectedSnapshot()-style callers
    // may come from several threads. If the device cannot be opened the driver
    // silently no-ops, so it is harmless on hardware without LEDs.
    class SpiLedStrip
    {
    public:
        explicit SpiLedStrip(int ledCount = 4);
        ~SpiLedStrip();

        SpiLedStrip(const SpiLedStrip &) = delete;
        SpiLedStrip &operator=(const SpiLedStrip &) = delete;

        // Returns true if the device opened. Safe to call even if it fails.
        bool Open(const std::string &device = "/dev/spidev0.0");
        void Close();
        bool IsOpen() const { return fd_ >= 0; }

        // Light exactly one LED white and turn the rest off. An index outside
        // [0, ledCount) turns every LED off. Redundant updates are suppressed.
        void ShowSingleWhite(int index);

        // Turn every LED off.
        void Clear() { ShowSingleWhite(-1); }

        // Global brightness, 0..31 (SK9822 5-bit current control). White at
        // full current is harsh, so a modest default is used.
        static constexpr uint8_t BRIGHTNESS = 8;

    private:
        void Write(const std::vector<std::array<uint8_t, 3>> &rgb);

        int fd_ = -1;
        int ledCount_;
        int lastIndex_ = -2; // last shown index, for redundant-update suppression
        std::mutex mutex_;
    };

    // Reads a kernel "gpio-keys" evdev device exposing four footswitches
    // (KEY_F1..KEY_F4) and translates presses into snapshot / preset / bank
    // actions:
    //
    //   switch 1..4 (single tap)          -> snapshot 1..4
    //   switch 1 + switch 2 (chord)       -> previous preset
    //   switch 3 + switch 4 (chord)       -> next preset
    //   switch 1 long-press (HOLD_MS)     -> previous bank
    //   switch 4 long-press (HOLD_MS)     -> next bank
    //
    // A chord is recognised by physical overlap: when the second switch of a
    // pair goes down while the first is still held, the preset action fires
    // immediately. This is independent of how far apart (in time) the two
    // presses land, which is what makes it robust for a foot stomping two
    // switches at once. A lone switch commits its snapshot when it is released
    // (a tap); the two outer switches instead fire their bank action if held
    // past the long-press threshold (in which case the later release is
    // ignored). A switch that has been consumed by a chord or long-press does
    // not also fire a snapshot.
    //
    // The handler owns a background thread that discovers and reads the input
    // device; if no matching device is present it simply idles and retries, so
    // it is harmless on hardware without footswitches.
    class FootswitchHandler
    {
    public:
        struct Callbacks
        {
            std::function<void(int snapshotIndex)> onSnapshot; // 0-based
            std::function<void()> onPreviousPreset;
            std::function<void()> onNextPreset;
            std::function<void()> onPreviousBank;
            std::function<void()> onNextBank;
        };

        FootswitchHandler(
            Callbacks callbacks,
            const std::string &deviceName = "gpio-keys");
        ~FootswitchHandler();

        FootswitchHandler(const FootswitchHandler &) = delete;
        FootswitchHandler &operator=(const FootswitchHandler &) = delete;

        void Start();
        void Stop();

        // Reflect the currently selected snapshot (0-based; -1 = none) on the
        // front-panel LEDs. Thread-safe; may be called from any thread.
        void SetSelectedSnapshot(int snapshotIndex);

    private:
        using clock = std::chrono::steady_clock;

        void ThreadProc();
        int OpenDevice();
        void ResetState();
        void HandleKey(int switchIndex, bool pressed, clock::time_point now);
        void FireExpiredHolds(clock::time_point now);
        int NextTimeoutMs(clock::time_point now) const;

        Callbacks callbacks_;
        std::string deviceName_;

        std::thread thread_;
        std::atomic<bool> stop_{false};
        int wakeEventFd_ = -1;

        // How long an outer switch (FS1/FS4) must be held to trigger a bank step.
        static constexpr std::chrono::milliseconds HOLD_WINDOW{700};

        static constexpr int SWITCH_COUNT = 4;
        enum class SwitchState
        {
            Released, // not pressed
            Down,     // pressed and held, not yet committed
            Consumed  // pressed and already acted upon (chord or long-press)
        };
        SwitchState state_[SWITCH_COUNT];
        clock::time_point pressTime_[SWITCH_COUNT]; // when the switch went down

        SpiLedStrip ledStrip_; // front-panel snapshot indicator LEDs
    };

} // namespace pipedal
