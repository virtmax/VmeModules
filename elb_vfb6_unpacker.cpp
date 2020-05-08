/*
    Data unpacker for the VME module: ELB VFB6 TDC.


    The firmware for the module can be found under:
    https://github.com/jobisoft/jTDC

    Copyright 2016 Maxim Singer

    License: MIT (https://opensource.org/licenses/MIT)

*/

#include "elb_vfb6_unpacker.h"

ElbVfb6Unpacker::ElbVfb6Unpacker()
{
    generator.seed(static_cast<std::default_random_engine::result_type>(clock()));

    for(size_t i = 0; i < nChannels; i++)
    {
        DNL_table.at(i).fill(10);
        calibration_table.at(i).fill(0);
    }

    triggerDuringBusy = 0;
    dataFifoOverflow = 0;
    oldTdcDataIgnored = 0;
    timeOverTheLimit = 0;

    lastEventTdcClockCounter = 0;
    tdcClockOverflows = 0;
    tdcClockInit = 0;
    eventCounter = 0;
    hitCounter = 0;

    hitsPerEventOverflow = 0;

    eventNrStart = -1;
    eventNrOffset = 0;
    eventNrLast = 0;

    triggerRequestStart = -1;
    triggerRequestOffset = 0;
    triggerRequestLast = 0;
}

ElbVfb6Unpacker::~ElbVfb6Unpacker()
{


}

int ElbVfb6Unpacker::analyseFileFormat(const std::vector<uint32_t> &raw_data,
                                    size_t start,
                                    size_t end,
                                    VFB6_EVENT_LIST &event_list,
                                    uint32_t window_size_clocks,
                                    bool return_events_with_no_hits)
{
    if(raw_data.size()==0)
    {
        return 0;
    }

    uint16_t data[2];

    uint32_t dnl_counter = 0;
    for(size_t di = start; di < end && di < raw_data.size();) // one loop = one trigger event
    {
		Vfb6Event vfb6event;
		vfb6event.eventNr = (raw_data[di] >> 16) & 0xFFFF;
        vfb6event.nHits = 0;
        const size_t eventSize = (raw_data[di] & 0x1FFF);
        ++di;

        dnl_counter = 0;

        size_t hit_counter = 0;
        //vfb6event.hittimes.clear();
	
        for(unsigned int i = 0; i < eventSize; ++i,++di)   // go through event data
        {
            data[0] = raw_data[di] & 0xFFFF;
            data[1] = (raw_data[di] >> 16) & 0xFFFF;

            for(unsigned int j = 0; j < 2; ++j)
            {
                const uint16_t d = data[j];
                const Datatype dt = getDataType(d);

                switch(dt)
                {
                    case hit:
                    {
                        if(vfb6event.nHits >= maxHitsPerEvent)
                        {
                            hitsPerEventOverflow++;
                            continue;
                        }

                        uint32_t ch = (d >> 8) & 0x7F;
                        uint64_t time_hp = d & 0xFF;

                        if(ch <= 96)
                        {
                            vfb6event.hits[vfb6event.nHits++] = d;
                            hit_counter++;
                            DNL_table.at(ch).at(time_hp)++;
                            dnl_counter++;
                        }
                    }
                    break;


                    case clockinfo:
                    {
                        const uint32_t clock =  d & 0x1FFF;

                        // there is a bug (feature?) in the TDC firmware.
                        // the first event outside the programed window
                        // will still reach the pipeline. sort it out by software here.
                        if(clock < 250 && clock < window_size_clocks) // 250*5 ns = 1250 ns
                        {
                            for (size_t l = vfb6event.nHits-hit_counter;
                                 l < vfb6event.nHits && l < maxHitsPerEvent; ++l)
							{
								const uint32_t time_21bit = (clock << 8) | (vfb6event.hits[l] & 0xFF);
								vfb6event.hits[l] = ((vfb6event.hits[l] & 0x7F00) << 13) | time_21bit;
							}
                            hitCounter += hit_counter;
							hit_counter = 0;
                        }
                        else
                            timeOverTheLimit++;
                    }
                    break;

                case header1:
                {
/*
                    const uint32_t h1_eventSize =  d & 0x1FFF;
                    if(h1_eventSize != eventSize)
                    {
                        std::cout <<"event h1 != fifo event size: " << h1_eventSize << " != " << eventSize << std::endl;
                    }
*/
                    const uint32_t timestamp_0_12 =  d & 0x1FFF;
                    vfb6event.timestamp1mu = timestamp_0_12;
                }
                    break;

                case header2:
                {
                    const int32_t trigger_request_0_12 =  d & 0x1FFF;
                    vfb6event.triggerReguest = trigger_request_0_12;
                }
                break;


                case filler:
                {
                }
                break;

                case trailer:
                {
                    if(((d >> 8) & 0x1) == 1)
                    {
                        ++triggerDuringBusy;
                        triggerDuringBusy = true;
                    }
                    if(((d >> 9) & 0x1) == 1)
                        ++dataFifoOverflow;
                    if(((d >> 10) & 0x1) == 1)
                        ++oldTdcDataIgnored;
/*
                    uint32_t t = (d & 0xFF);
                    if((vfb6event.eventNr & 0xFF) != t)
                    {
                        std::cout <<"wrong event number in trailer: " << (vfb6event.eventNr & 0xFF) << " t=" << t << std::endl;
                    }*/

                    //const uint32_t timestamp_13_21 =  d & 0xFF;
                    //vfb6event.timestamp1mu = timestamp_13_21;
                    //vfb6event.timestamp1mu = (timestamp_13_21 << 13) | vfb6event.timestamp1mu;
                }
                break;

                default:
                    break;
                }
            }


        }

        eventCounter++;
        if(return_events_with_no_hits || vfb6event.nHits > 0)
        {
            event_list.push_back(vfb6event);
        }
    }

    return 0;
}

std::array<ElbVfb6Unpacker::CALIBRATION_TABLE, ElbVfb6Unpacker::nChannels> ElbVfb6Unpacker::getCalibrationTable()
{
    // integrate the DNL table to get the calibration curve
    for(size_t ch = 0; ch < nChannels; ch++)
    {
            calibration_table.at(ch) = getCalibrationTable(ch);
    }

    return calibration_table;
}

ElbVfb6Unpacker::CALIBRATION_TABLE ElbVfb6Unpacker::getCalibrationTable(size_t channel)
{
    double full_sum = std::accumulate(DNL_table.at(channel).begin(), DNL_table.at(channel).end(), 0);

    for(size_t j = 0; full_sum > 0 && j < 100; j++)
    {
        const double sum1 = std::accumulate(DNL_table.at(channel).begin(), DNL_table.at(channel).begin()+j, 0);
        const double sum2 = std::accumulate(DNL_table.at(channel).begin()+128, DNL_table.at(channel).begin()+128+j, 0);
        calibration_table.at(channel).at(j) = ((sum1+sum2)/full_sum);
    }

    return calibration_table.at(channel);
}

double ElbVfb6Unpacker::getTime(uint32_t hitcode,
                                const CALIBRATION_TABLE& calibr_table,
                                TimeInterpolation interpolation)
{
    const uint32_t clockcounts = (hitcode >> 7) & 0x3FFF;
    const uint32_t time_hp = hitcode & 0x7F;

    double time_in_ns = static_cast<double>(clockcounts);

	if(interpolation == TimeInterpolation::None)
	{
		time_in_ns += calibr_table.at(time_hp);
	}
	else if(interpolation == TimeInterpolation::Linear)
    {
        time_in_ns += 0.5*(calibr_table.at(time_hp) + calibr_table.at(time_hp + 1));
    }
	else if(interpolation == TimeInterpolation::Random)
	{
		std::uniform_real_distribution<double> distribution(calibr_table.at(time_hp), calibr_table.at(time_hp + 1));
		time_in_ns += distribution(generator);
	}

    return time_in_ns*2.5;
}

double ElbVfb6Unpacker::getTime(uint32_t hitcode, TimeInterpolation interpolation)
{
    return getTime(hitcode, calibration_table.at(getChannel(hitcode)), interpolation);
}

uint32_t ElbVfb6Unpacker::getChannel(uint32_t hitcode) const
{
    return (hitcode >> 21) & 0x7F;
}

std::pair<uint32_t, double> ElbVfb6Unpacker::getTimeAndChannel(uint32_t hitcode, const CALIBRATION_TABLE& calibr_table, TimeInterpolation interpolation)
{
    return {getChannel(hitcode), getTime(hitcode, calibr_table, interpolation)};
}

std::pair<uint32_t, double> ElbVfb6Unpacker::getTimeAndChannel(uint32_t hitcode, TimeInterpolation interpolation)
{
    return {getChannel(hitcode), getTime(hitcode, calibration_table.at(getChannel(hitcode)), interpolation)};
}

int64_t ElbVfb6Unpacker::getExtendedEventNr(int32_t tdcEventNr)
{
    if(eventNrStart < 0)
    {
        eventNrStart = tdcEventNr;
    }
    else
    {
        if(tdcEventNr < eventNrLast)
        {
            eventNrOffset += 0xFFFF;
            eventNrOffset += 1;
        }
    }

    eventNrLast = tdcEventNr;
    return (eventNrOffset + tdcEventNr) - eventNrStart;
}

int64_t ElbVfb6Unpacker::getExtendedTriggerRequestNr(int32_t triggerRequestNr)
{
    if(triggerRequestStart < 0)
    {
        triggerRequestStart = triggerRequestNr;
    }
    else
    {
        if(triggerRequestNr < triggerRequestLast)
        {
            triggerRequestOffset += 0x1FFF;
            triggerRequestOffset += 1;
        }
    }

    triggerRequestLast = triggerRequestNr;
    return (triggerRequestOffset + triggerRequestNr) - triggerRequestStart;
}

void ElbVfb6Unpacker::reset()
{
    for(size_t ch = 0; ch < nChannels; ch++)
    {
        DNL_table.at(ch).fill(0);
        calibration_table.at(ch).fill(0);
    }

}

ElbVfb6Unpacker::Datatype ElbVfb6Unpacker::getDataType(uint16_t data)
{
    Datatype res;

    if ((data >> 15) == 0x0)
    {
        res = hit;
    }
    else if((data >> 13) == 0x4)
    {
        res = clockinfo;
    }
    else if((data >> 13) == 0x6)
    {
        res = header1;
    }
    else if((data >> 13) == 0x7)
    {
        res = header2;
    }
    else if((data >> 11) == 0x16)
    {
        res = filler;
    }
    else if((data >> 12) == 0xA)
    {
        res = trailer;
    }
    else
    {
        res = unknown;
    }

    return res;
}
