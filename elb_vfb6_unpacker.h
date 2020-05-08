/*
    Data unpacker for the VME module: ELB VFB6 TDC.

    
    The firmware for the module can be found under:
    https://github.com/jobisoft/jTDC

    Copyright 2016 Maxim Singer

    License: MIT (https://opensource.org/licenses/MIT)

*/

#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <memory>
#include <algorithm>
#include <numeric>
#include <array>
#include <random>
#include <time.h>
#include <map>


class ElbVfb6Unpacker
{
public:
    ElbVfb6Unpacker();
    ~ElbVfb6Unpacker();

    const static uint32_t nChannels = 128;      // defined by firmware
    const static uint32_t nTimeBins = 129;
    const static size_t maxHitsPerEvent = 256;  // maximum expected hits per one event

    enum TimeInterpolation {None, Linear, Random};

    class Vfb6Event
    {
    public:
        Vfb6Event() : eventNr(-1), timestamp1mu(0),
                        triggerReguest(-1), nHits(0), triggerDuringBusy(false)
        {
            hits.fill(0);
        }

        long eventNr;
        uint64_t timestamp1mu;
        int32_t triggerReguest;
        
        uint32_t nHits;
        std::array<uint32_t, maxHitsPerEvent> hits;

        bool triggerDuringBusy;
    };

    // < eventNr, list with hit times for each channel>
    typedef std::vector<Vfb6Event> VFB6_EVENT_LIST;
    typedef std::array<double, nTimeBins> CALIBRATION_TABLE;

    int analyseFileFormat(const std::vector<uint32_t> &raw_data,
                          size_t start,
                          size_t end,
                          VFB6_EVENT_LIST &event_list,
                          uint32_t window_size_clocks = 250,
                          bool return_events_with_no_hits=false);

    // return time in nanoseconds
    double getTime(uint32_t hitcode, const CALIBRATION_TABLE& calibr_table, TimeInterpolation interpolation);
    double getTime(uint32_t hitcode, TimeInterpolation interpolation);

    uint32_t getChannel(uint32_t hitcode) const;

    std::pair<uint32_t, double> getTimeAndChannel(uint32_t hitcode, const CALIBRATION_TABLE& calibr_table, TimeInterpolation interpolation);
    std::pair<uint32_t, double> getTimeAndChannel(uint32_t hitcode, TimeInterpolation interpolation);

    // create a successive event number, bigger than 16 bit event nr given by the tdc firmware
    // detects an event number overflow
    int64_t getExtendedEventNr(int32_t tdcEventNr);

    int64_t getExtendedTriggerRequestNr(int32_t triggerRequestNr);

    std::array<std::array<uint32_t, nTimeBins*2>, nChannels> getDNLTable() const { return DNL_table;}
    std::array<CALIBRATION_TABLE, nChannels> getCalibrationTable();
    CALIBRATION_TABLE getCalibrationTable(size_t channel);

    // clears DNL_table and calibration_table
    void reset();

    size_t getHitCounter() const { return hitCounter; }
    size_t getEventCounter() const { return eventCounter; }
    size_t getTriggerDuringBusyCount() const { return triggerDuringBusy; }
    size_t getDataFifoOverflowCount() const { return dataFifoOverflow; }
    size_t getOldTDCDataIgnoreCount() const { return oldTdcDataIgnored; }
    size_t getTimeOverTheLimitCount() const { return timeOverTheLimit; }
    size_t getHitsPerEventOverflow() const { return hitsPerEventOverflow; }

private:
    enum Datatype {header1=0, header2, hit, clockinfo, filler, trailer, unknown};

    Datatype getDataType(uint16_t data);

    std::array<std::array<uint32_t, nTimeBins*2>, nChannels>  DNL_table;           // differential non-linearity (DNL) table
    std::array<CALIBRATION_TABLE, nChannels>    calibration_table;                 // DNL-table integrated and normalized to 1

    size_t eventCounter;
    size_t hitCounter;

    int64_t tdcClockInit;
    int64_t lastEventTdcClockCounter;
    int32_t tdcClockOverflows;

    // counters for errors
    size_t triggerDuringBusy;
    size_t dataFifoOverflow;
    size_t oldTdcDataIgnored;
    size_t timeOverTheLimit;

    size_t hitsPerEventOverflow;

    std::default_random_engine generator;

    int32_t eventNrStart;
    int32_t eventNrLast;
    int32_t eventNrOffset;

    int32_t triggerRequestStart;
    int32_t triggerRequestLast;
    int32_t triggerRequestOffset;
};
