#ifndef DL_TLS_PACKET_SCHEDULER_H_
#define DL_TLS_PACKET_SCHEDULER_H_

#include "downlink-packet-scheduler.h"


class DL_TLS_PacketScheduler : public DownlinkPacketScheduler {
public:
	DL_TLS_PacketScheduler();
	virtual ~DL_TLS_PacketScheduler();

	virtual void DoSchedule (void);
	virtual void DoStopSchedule (void);
	void RunControlLaw ();
	virtual double ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel);
	void Select_RT_FlowsToSchedule ();
	void Select_NRT_FlowsToSchedule ();
	void UpdateDataToTransmitAndAverageDataRate (void);

	void RBsAllocation (int subframe);
	void RBsAllocationForFrame_RT();
	void RBsAllocationForFrame_NRT();
	bool If_Qsch_None ();



private:
	bool m_runControlLaw;
	int m_subFrameCounter;
	int numofRT;
	std::vector<int> Qsch;
	std::vector<int> Rem_RB;

};

#endif /* DL_TLS_PACKET_SCHEDULER_H_ */