#include "dl-tls-packet-scheduler.h"
#include "../mac-entity.h"
#include "../../packet/Packet.h"
#include "../../packet/packet-burst.h"
#include "../../../device/NetworkNode.h"
#include "../../../flows/radio-bearer.h"
#include "../../../protocolStack/rrc/rrc-entity.h"
#include "../../../flows/application/Application.h"
#include "../../../device/ENodeB.h"
#include "../../../protocolStack/mac/AMCModule.h"
#include "../../../phy/lte-phy.h"
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../flows/QoS/QoSForFLS.h"
#include "../../../flows/MacQueue.h"
#include "../../../utility/eesm-effective-sinr.h"
//#define SCHEDULER_DEBUG


static double minvalue=-100000;
static double maxvalue=100000;
//int OUT=0;
static int getmaxindex(double* a,int N){
	int index=-1;double tmp=minvalue;
	for(int i=0;i<N;i++){
		if(a[i]>tmp){
			tmp=a[i];
			index=i;
		}
	}
	return index;
}

/*static int getmaxindex(double *a[],int row,int N){
	int index=-1;double tmp=minvalue;
	for(int i=0;i<N;i++){
		if(a[row][i]>=tmp){
			tmp=a[row][i];
			index=i;
		}
	}
	return index;
}*/



DL_TLS_PacketScheduler::DL_TLS_PacketScheduler()
{
  SetMacEntity (0);
  CreateFlowsToSchedule ();
  m_runControlLaw = true;
  m_subFrameCounter = 0;
  numofRT=0;
}

DL_TLS_PacketScheduler::~DL_TLS_PacketScheduler()
{
	Destroy();
}

double
DL_TLS_PacketScheduler::ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel)
{
  double metric;

  if ((bearer->GetApplication ()->GetApplicationType () == Application::APPLICATION_TYPE_INFINITE_BUFFER)
	  ||
	  (bearer->GetApplication ()->GetApplicationType () == Application::APPLICATION_TYPE_CBR))
    {
	  metric = (spectralEfficiency * 180000.)
				/
	    	    bearer->GetAverageTransmissionRate();
    }
  else
    {

     QoSForFLS *qos = (QoSForFLS*) bearer->GetQoSParameters ();

     //double a = (-log10 (qos->GetDropProbability())) / qos->GetMaxDelay ();
     double HOL = bearer->GetHeadOfLinePacketDelay ();

	 /*metric = (a * HOL)
			 *
			 ((spectralEfficiency * 180000.)
			 /
			 bearer->GetAverageTransmissionRate ());*/
     	//metric=(HOL/qos->GetMaxDelay())*((spectralEfficiency*180000.)/bearer->GetAverageTransmissionRate());
     	metric=(HOL/qos->GetMaxDelay())*(spectralEfficiency);

    }

  return metric;
}

bool DL_TLS_PacketScheduler::If_Qsch_None()
{
	for(std::vector<int>::iterator it=Qsch.begin();it!=Qsch.end();it++){
		if(*(it)==1)
			return false;
	}
	return true;
}



void
DL_TLS_PacketScheduler::DoSchedule ()
{
#ifdef SCHEDULER_DEBUG
	std::cout << "Start DL packet (TLS) scheduler for node "
	<< GetMacEntity ()->GetDevice ()->GetIDNetworkNode()<< std::endl;
#endif
	UpdateDataToTransmitAndAverageDataRate ();
	CheckForDLDropPackets ();

	if (m_runControlLaw)
	{
		Qsch.clear();Rem_RB.clear();
  		ClearFlowsToSchedule ();
		RunControlLaw ();
		int nbOfCHs = GetMacEntity ()->GetDevice ()->GetPhy ()->GetBandwidthManager ()->GetDlSubChannels ().size ();
 		for(int ii=0;ii<nbOfCHs;ii++){
 			Qsch.push_back(1);
 			Rem_RB.push_back(10);
 		}
 		Select_RT_FlowsToSchedule();
		if((numofRT=GetFlowsToSchedule()->size())!=0)
		{
			RBsAllocationForFrame_RT();
		}
		if(!If_Qsch_None())
		{
			Select_NRT_FlowsToSchedule ();
			if ((GetFlowsToSchedule ()->size()-numofRT) != 0)
			{
				RBsAllocationForFrame_NRT();
			}
		}	  
	}
	RBsAllocation(m_subFrameCounter);
	StopSchedule();
	m_subFrameCounter ++;
	if (m_subFrameCounter == 10)
	{
		m_runControlLaw = true;
		m_subFrameCounter = 0;
	}


}

void
DL_TLS_PacketScheduler::DoStopSchedule (void)
{
#ifdef SCHEDULER_DEBUG
	std::cout << "\t Do Stop Schedule (TLS) Creating Packet Burst" << std::endl;
#endif

	PacketBurst* pb = new PacketBurst ();

  //Create Packet Burst
	FlowsToSchedule *flowsToSchedule = GetFlowsToSchedule ();

	for (FlowsToSchedule::iterator it = flowsToSchedule->begin (); it != flowsToSchedule->end (); it++)
	{
		FlowToSchedule *flow = (*it);

		int availableBytes = flow->GetAllocatedBits ()/8;

		if (availableBytes > 0)
		{
			flow->GetBearer ()->UpdateTransmittedBytes (availableBytes);
			flow->SetAllocatedBits(0);
#ifdef SCHEDULER_DEBUG
			std::cout << "\t  --> add packets for flow "
			<< flow->GetBearer ()->GetApplication ()->GetApplicationID () << std::endl;
#endif

	      //flow->GetBearer ()->GetMacQueue ()->PrintQueueInfo ();
			RlcEntity *rlc = flow->GetBearer ()->GetRlcEntity ();
			PacketBurst* pb2 = rlc->TransmissionProcedure (availableBytes);


#ifdef SCHEDULER_DEBUG
			std::cout << "\t\t  nb of packets: " << pb2->GetNPackets () << std::endl;
#endif
			if (pb2->GetNPackets () > 0)
			{
				std::list<Packet*> packets = pb2->GetPackets ();
				std::list<Packet* >::iterator it;
				for (it = packets.begin (); it != packets.end (); it++)
				{
#ifdef SCHEDULER_DEBUG
					std::cout << "\t\t  added packet of bytes " << (*it)->GetSize () << std::endl;
	    		  //(*it)->Print ();
#endif

					Packet *p = (*it);
					pb->AddPacket (p->Copy ());
				}
			}
			delete pb2;
		}
		else
		{}
}
  //SEND PACKET BURST
#ifdef SCHEDULER_DEBUG
if (pb->GetNPackets () == 0)
	std::cout << "\t Send only reference symbols" << std::endl;
#endif

GetMacEntity ()->GetDevice ()->SendPacketBurst (pb);
#ifdef SCHEDULER_DEBUG
for (FlowsToSchedule::iterator it = flowsToSchedule->begin (); it != flowsToSchedule->end (); it++){
		 (*it)->GetBearer()->GetMacQueue()->PrintQueueInfo ();
}
#endif

}


void DL_TLS_PacketScheduler::RBsAllocationForFrame_RT(){
	#ifdef SCHEDULER_DEBUG
		std::cout << " ---- DownlinkPacketScheduler::RBsAllocationForFrame_RT";
	#endif
	FlowsToSchedule* flows = GetFlowsToSchedule ();
 	int nbOfCHs = GetMacEntity ()->GetDevice ()->GetPhy ()->GetBandwidthManager ()->GetDlSubChannels ().size ();
 	/*for(int ii=0;ii<nbOfCHs;ii++){
 		Qsch.push_back(1);
 		Rem_RB.push_back(10);
 	}*/

  //create a matrix of flow metrics
  double metrics[nbOfCHs][flows->size ()];
  for (int i = 0; i < nbOfCHs; i++)
    {
	  for (int j = 0; j < flows->size (); j++)
	    {
		  metrics[i][j] = ComputeSchedulingMetric (flows->at (j)->GetBearer (),
				                                   flows->at (j)->GetSpectralEfficiency ().at (i),
	    		                                   i);
	    }
    }

  #ifdef SCHEDULER_DEBUG
    std::cout << ", available subChannels " << nbOfCHs << ", flows " << flows->size () << std::endl;
    for (int ii = 0; ii < flows->size (); ii++)
    {
    	std::cout << "\t metrics for flow "
    	<< flows->at (ii)->GetBearer ()->GetApplication ()->GetApplicationID () << ":";
    	for (int jj = 0; jj < nbOfCHs; jj++)
    	{
    		std::cout << " " << metrics[jj][ii];
    	}
    	std::cout << std::endl;
    }
  #endif


    int l_iScheduledFlows = 0;
    int flowid;
    FlowToSchedule* scheduledFlow;
    AMCModule *amc = GetMacEntity ()->GetAmcModule ();

    while(!If_Qsch_None()){
  
    	if(l_iScheduledFlows==flows->size())
    		break;
    	int ii;
    	for(ii=0;ii<nbOfCHs;ii++){
    		if(l_iScheduledFlows==flows->size())
    			break;
    		if(Qsch[ii]!=0){
    			flowid=getmaxindex(metrics[ii],flows->size());
    			//flowid=getmaxindex(metrics,ii,flows->size());
    			scheduledFlow=flows->at(flowid);
    			double sinr = amc->GetSinrFromCQI (scheduledFlow->GetCqiFeedbacks ().at (ii));
          		int mcs = amc->GetMCSFromCQI (amc->GetCQIFromSinr (sinr));

          		int jj;
          		int tmpdata=0;
    			for(jj=10-Rem_RB[ii];jj<10;jj++){
    				//Rem_RB[ii]--;
    				scheduledFlow->GetListOfAllocatedRBsForFrame()->push_back(jj*nbOfCHs+ii);
    				#ifdef SCHEDULER_DEBUG
          				std::cout << "\t *** RB line " <<ii<<" col "<<jj << " assigned to the "
                  		" flow " << scheduledFlow->GetBearer ()->GetApplication ()->GetApplicationID ()
                  		<< " DST "<<scheduledFlow->GetBearer ()->GetDestination()->GetIDNetworkNode()<<std::endl;
					#endif
    				scheduledFlow->GetListOfSelectedMCSForFrame()->push_back(mcs);
    				tmpdata+=amc->GetTBSizeFromMCS (mcs, 1);
    				if(tmpdata>=scheduledFlow->GetDataToTransmit()*8)
    					break;
    				/*if(amc->GetTBSizeFromMCS (mcs, jj+Rem_RB[ii]-9)>=scheduledFlow->GetDataToTransmit()*8)
    					break;*/
    			}
    			if(jj<10){
    				l_iScheduledFlows++;
    				scheduledFlow->SetDataToTransmit(0);
    				for(int kk=0;kk<nbOfCHs;kk++)
    					metrics[kk][flowid]=minvalue;
    				Rem_RB[ii]=9-jj;
    				if(Rem_RB[ii]==0)
    					Qsch[ii]=0;
    			}
    			else{
    				Qsch[ii]=0;
    				//scheduledFlow->SetDataToTransmit(scheduledFlow->GetDataToTransmit()-amc->GetTBSizeFromMCS(mcs,Rem_RB[ii])/8);
    				scheduledFlow->SetDataToTransmit(scheduledFlow->GetDataToTransmit()-tmpdata/8);
    				Rem_RB[ii]=0;
    			}
    		}
    	}
    }
    #ifdef SCHEDULER_DEBUG
    	for (FlowsToSchedule::iterator it = flows->begin (); it != flows->end (); it++){
      		FlowToSchedule *flow = (*it);
      		double bitsToTransmit=0;
      		/*int RB_num=1,i=0;
      		while(i<flow->GetListOfAllocatedRBsForFrame()->size()){
      			if(i+1<flow->GetListOfAllocatedRBsForFrame()->size() && flow->GetListOfSelectedMCSForFrame()->at(i)==flow->GetListOfSelectedMCSForFrame()->at(i+1) && RB_num<10){
      				i++;RB_num++;
      			}
      			else{
      				bitsToTransmit+=amc->GetTBSizeFromMCS(flow->GetListOfSelectedMCSForFrame()->at(i),RB_num);
      				i++;RB_num=1;
      			}
      		}*/
      		for(int i=0;i<flow->GetListOfAllocatedRBsForFrame()->size();i++){
      			bitsToTransmit+=amc->GetTBSizeFromMCS(flow->GetListOfSelectedMCSForFrame()->at(i),1);
      		}
		 	 std::cout << "\t\t --> flow "	<< flow->GetBearer ()->GetApplication ()->GetApplicationID ()
				  << " has been scheduled: " <<
				  "\n\t\t\t nb of RBs " << flow->GetListOfAllocatedRBsForFrame()->size () <<
				  "\n\t\t\t bitsToTransmit " << bitsToTransmit
				  << std::endl;
				}
	#endif
    /*if(l_iScheduledFlows==0)
    	OUT++;*/

}


void DL_TLS_PacketScheduler::RBsAllocationForFrame_NRT(){
	#ifdef SCHEDULER_DEBUG
		std::cout << " ---- DownlinkPacketScheduler::RBsAllocationForFrame_NRT";
	#endif

	FlowsToSchedule* flows = GetFlowsToSchedule ();
	int nbOfCHs = GetMacEntity ()->GetDevice ()->GetPhy ()->GetBandwidthManager ()->GetDlSubChannels ().size ();

  //create a matrix of flow metrics
	double metrics[nbOfCHs][flows->size ()-numofRT];
	int size=flows->size()-numofRT;
	for (int i = 0; i < nbOfCHs; i++)
	{
		for (int j = 0; j < size; j++)
		{
			metrics[i][j] = ComputeSchedulingMetric (flows->at (j+numofRT)->GetBearer (),
				flows->at (j+numofRT)->GetSpectralEfficiency ().at (i),
				i);
		}
	}
	int l_iScheduledFlows = 0;
	int flowid;
	FlowToSchedule* scheduledFlow;
	AMCModule *amc = GetMacEntity ()->GetAmcModule ();

	while(!If_Qsch_None()){
		if(l_iScheduledFlows==size)
			break;
		for(int ii=0;ii<nbOfCHs;ii++){
			if(l_iScheduledFlows==size)
    			break;
			if(Qsch[ii]!=0){
				flowid=getmaxindex(metrics[ii],size)+numofRT;
				//flowid=getmaxindex(metrics,ii,size)+numofRT;
				scheduledFlow=flows->at(flowid);
				double sinr = amc->GetSinrFromCQI (scheduledFlow->GetCqiFeedbacks ().at (ii));
				int mcs = amc->GetMCSFromCQI (amc->GetCQIFromSinr (sinr));
          		//int transportBlockSize = amc->GetTBSizeFromMCS (mcs, scheduledFlow->GetListOfAllocatedRBs ()->size ());
          		int jj;
          		int tmpdata=0;
				for(jj=10-Rem_RB[ii];jj<10;jj++){
    				//Rem_RB[ii]--;
					scheduledFlow->GetListOfAllocatedRBsForFrame()->push_back(jj*nbOfCHs+ii);
					scheduledFlow->GetListOfSelectedMCSForFrame()->push_back(mcs);
					tmpdata+=amc->GetTBSizeFromMCS (mcs, 1);
    				if(tmpdata>=scheduledFlow->GetDataToTransmit()*8)
    					break;
					/*if(amc->GetTBSizeFromMCS (mcs, jj+Rem_RB[ii]-9)>=scheduledFlow->GetDataToTransmit()*8)
						break;*/
				}
				if(jj<10){
					l_iScheduledFlows++;
					scheduledFlow->SetDataToTransmit(0);
					for(int kk=0;kk<nbOfCHs;kk++)
						metrics[kk][flowid-numofRT]=minvalue;
					Rem_RB[ii]=9-jj;
					if(Rem_RB[ii]==0)
						Qsch[ii]=0;
				}
				else{
					Qsch[ii]=0;
					scheduledFlow->SetDataToTransmit(scheduledFlow->GetDataToTransmit()-tmpdata/8);
					//scheduledFlow->SetDataToTransmit(scheduledFlow->GetDataToTransmit()-amc->GetTBSizeFromMCS(mcs,Rem_RB[ii])/8);
					Rem_RB[ii]=0;
				}
			}
		}
	}

}

void DL_TLS_PacketScheduler::RBsAllocation(int subframe){
	#ifdef SCHEDULER_DEBUG
		std::cout << " ---- DownlinkPacketScheduler::RBsAllocation";
	#endif
	
	int i,j;
	AMCModule *amc = GetMacEntity ()->GetAmcModule ();
	FlowToSchedule* scheduledFlow;
	FlowsToSchedule* flows = GetFlowsToSchedule ();
	int nbOfCHs = GetMacEntity ()->GetDevice ()->GetPhy ()->GetBandwidthManager ()->GetDlSubChannels ().size ();
	PdcchMapIdealControlMessage *pdcchMsg = new PdcchMapIdealControlMessage ();
	for(i=0;i<flows->size();i++){
		scheduledFlow=flows->at(i);
		for(j=0;j<scheduledFlow->GetListOfAllocatedRBsForFrame()->size();j++){
			if(scheduledFlow->GetListOfAllocatedRBsForFrame()->at(j)/nbOfCHs==subframe){
				scheduledFlow->GetListOfAllocatedRBs()->push_back(scheduledFlow->GetListOfAllocatedRBsForFrame()->at(j)%nbOfCHs);
				scheduledFlow->GetListOfSelectedMCS()->push_back(scheduledFlow->GetListOfSelectedMCSForFrame()->at(j));
				#ifdef SCHEDULER_DEBUG
          		  std::cout << "\t *** RB " << scheduledFlow->GetListOfAllocatedRBsForFrame()->at(j)%nbOfCHs << " assigned to the "
                  " flow " << scheduledFlow->GetBearer ()->GetApplication ()->GetApplicationID ()
                  << " DST "<<scheduledFlow->GetBearer ()->GetDestination()->GetIDNetworkNode()<<std::endl;
			    #endif
			}
		}
		int transportBlockSize=0;
		double bitsToTransmit = 0;
		if (scheduledFlow->GetListOfAllocatedRBs ()->size () > 0)
		{
          //this flow has been scheduled
			for(int k=0;k<scheduledFlow->GetListOfAllocatedRBs()->size();k++){
				transportBlockSize+=amc->GetTBSizeFromMCS (scheduledFlow->GetListOfSelectedMCS()->at(k), 1);
			}
          //define the amount of bytes to transmit
			bitsToTransmit = transportBlockSize;
			scheduledFlow->SetAllocatedBits (bitsToTransmit);
		}

			#ifdef SCHEDULER_DEBUG
		  		std::cout << "\t\t --> flow "	<< scheduledFlow->GetBearer ()->GetApplication ()->GetApplicationID ()
				  << " has been scheduled: " <<
				  "\n\t\t\t nb of RBs " << scheduledFlow->GetListOfAllocatedRBs ()->size () <<
				  "\n\t\t\t tbs " << transportBlockSize <<
				  "\n\t\t\t bitsToTransmit " << bitsToTransmit<<
				  "\n\t\t\t AllocatedBits " << scheduledFlow->GetAllocatedBits();
				  << std::endl;
			#endif
		//create PDCCH messages
		for (int rb = 0; rb < scheduledFlow->GetListOfAllocatedRBs ()->size (); rb++ )
		{
			pdcchMsg->AddNewRecord (PdcchMapIdealControlMessage::DOWNLINK,
				scheduledFlow->GetListOfAllocatedRBs ()->at (rb),
				scheduledFlow->GetBearer ()->GetDestination (),
				scheduledFlow->GetListOfSelectedMCS()->at(rb));
		}
		scheduledFlow->GetListOfAllocatedRBs()->clear();
		scheduledFlow->GetListOfSelectedMCS()->clear();

	}
	if (pdcchMsg->GetMessage()->size () > 0)
		{
			GetMacEntity ()->GetDevice ()->GetPhy ()->SendIdealControlMessage (pdcchMsg);
		}
		delete pdcchMsg;
}

void
DL_TLS_PacketScheduler::Select_RT_FlowsToSchedule ()
{
  /*Qsch.clear();Rem_RB.clear();
  ClearFlowsToSchedule ();*/

#ifdef SCHEDULER_DEBUG
	std::cout << "\t Select RT Flows to schedule" << std::endl;
#endif


	RrcEntity *rrc = GetMacEntity ()->GetDevice ()->GetProtocolStack ()->GetRrcEntity ();
  	RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer ();

  	for (std::vector<RadioBearer* >::iterator it = bearers->begin (); it != bearers->end (); it++)
		{
	  	//SELECT FLOWS TO SCHEDULE
	  	RadioBearer *bearer = (*it);
	  	QoSForFLS* qos = (QoSForFLS*) bearer->GetQoSParameters ();

	  	if (bearer->HasPackets ()
			  	&&
			  	bearer->GetDestination ()->GetNodeState () == NetworkNode::STATE_ACTIVE
			  	&&
			  	qos->GetDataToTransmit () > 0
			  	&&
			      	(bearer->GetApplication ()->GetApplicationType() == Application::APPLICATION_TYPE_TRACE_BASED
			  	  	||
                  	bearer->GetApplication ()->GetApplicationType() == Application::APPLICATION_TYPE_VOIP ))
			{
		  	//data to transmit
		  	int dataToTransmit = qos->GetDataToTransmit ();

		  //compute spectral efficiency
		  	ENodeB *enb = (ENodeB*) GetMacEntity ()->GetDevice ();
		  	ENodeB::UserEquipmentRecord *ueRecord = enb->GetUserEquipmentRecord (bearer->GetDestination ()->GetIDNetworkNode ());
		  	std::vector<double> spectralEfficiency;
		  	std::vector<int> cqiFeedbacks = ueRecord->GetCQI ();
		  	int numberOfCqi = cqiFeedbacks.size ();
		  for (int i = 0; i < numberOfCqi; i++)
			{
			  double sEff = GetMacEntity ()->GetAmcModule ()->GetEfficiencyFromCQI (cqiFeedbacks.at (i));
			  spectralEfficiency.push_back (sEff);
			}

		  //create flow to schedule record
		  InsertFlowToSchedule(bearer, dataToTransmit, spectralEfficiency, cqiFeedbacks);
		}
	  else
	    {}
	}
}


void
DL_TLS_PacketScheduler::Select_NRT_FlowsToSchedule ()
{

#ifdef SCHEDULER_DEBUG
	std::cout << "\t Select NRT Flows to schedule" << std::endl;
#endif

  RrcEntity *rrc = GetMacEntity ()->GetDevice ()->GetProtocolStack ()->GetRrcEntity ();
  RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer ();


  for (std::vector<RadioBearer* >::iterator it = bearers->begin (); it != bearers->end (); it++)
	{
	  //SELECT FLOWS TO SCHEDULE
	  RadioBearer *bearer = (*it);
	  if (bearer->HasPackets ()
			  &&
			  bearer->GetDestination ()->GetNodeState () == NetworkNode::STATE_ACTIVE
			  &&
			      (bearer->GetApplication ()->GetApplicationType() == Application::APPLICATION_TYPE_CBR
			       ||
			       bearer->GetApplication ()->GetApplicationType() == Application::APPLICATION_TYPE_INFINITE_BUFFER ))
		{

		  //compute data to transmit
		  int dataToTransmit;

		  if (bearer->GetApplication ()->GetApplicationType() == Application::APPLICATION_TYPE_INFINITE_BUFFER)
		    {
			  dataToTransmit = 100000;
		    }
		  else
		    {
			  dataToTransmit = bearer->GetQueueSize ();
		    }

		  //compute spectral efficiency
		  ENodeB *enb = (ENodeB*) GetMacEntity ()->GetDevice ();
		  ENodeB::UserEquipmentRecord *ueRecord = enb->GetUserEquipmentRecord (bearer->GetDestination ()->GetIDNetworkNode ());
		  std::vector<double> spectralEfficiency;
		  std::vector<int> cqiFeedbacks = ueRecord->GetCQI ();
		  int numberOfCqi = cqiFeedbacks.size ();
		  for (int i = 0; i < numberOfCqi; i++)
			{
			  double sEff = GetMacEntity ()->GetAmcModule ()->GetEfficiencyFromCQI (cqiFeedbacks.at (i));
			  spectralEfficiency.push_back (sEff);
			}

		  //create flow to schedule record
		  InsertFlowToSchedule(bearer, dataToTransmit, spectralEfficiency, cqiFeedbacks);
		}
	  else
	    {}
	}
}


void
DL_TLS_PacketScheduler::RunControlLaw ()
{
  m_runControlLaw = false;

#ifdef SCHEDULER_DEBUG
  std::cout << " TLS_DEBUG:Run Control LOW"<< std::endl;
#endif
  RrcEntity *rrc = GetMacEntity ()->GetDevice ()->GetProtocolStack ()->GetRrcEntity ();
  RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer ();

  for (std::vector<RadioBearer* >::iterator it = bearers->begin (); it != bearers->end (); it++)
	{

	  RadioBearer *bearer = (*it);
	  if (bearer->GetApplication ()->GetApplicationType() == Application::APPLICATION_TYPE_TRACE_BASED
			  ||
			  bearer->GetApplication ()->GetApplicationType() == Application::APPLICATION_TYPE_VOIP )
		{

#ifdef SCHEDULER_DEBUG
		  bearer->GetMacQueue ()->PrintQueueInfo ();
#endif

		  //Frame Level Scheduler Control Low!!!
		  QoSForFLS* qos = (QoSForFLS*) bearer->GetQoSParameters ();

		  int queueSize = bearer->GetQueueSize ();
	      int* q = qos->GetQ ();
	      int* u = qos->GetU ();
	      double* c = qos->GetFilterCoefficients();
	      int M = qos->GetNbOfCoefficients ();

	      double dataToTransmit = ((double)(1-c[2]) * queueSize);
	      for (int i = 0; i < M-1; i++)
	        {
	    	  dataToTransmit += (double)q[i]*c[i+2];
	        }
	      for (int i = 0; i < M-2; i++)
			{
	      	  dataToTransmit -= (double)q[i]*c[i+3];
			}
	      for (int i = 0; i < M-1; i++)
			{
			  dataToTransmit -= (double)u[i]*c[i+2];
			}

	      if (dataToTransmit < 0)
	        {
	    	  dataToTransmit = 0;
	        }


	      if (bearer->HasPackets())
	        {
	    	  int minData = 8 + bearer->GetHeadOfLinePacketSize ();
    		  int maxData = bearer->GetMacQueue ()->GetByte (dataToTransmit);
#ifdef SCHEDULER_DEBUG
	      std::cout << "\t dataToTransmit " << dataToTransmit << " minData " << minData  << " maxData " <<  maxData  << std::endl;
#endif
    		  if (dataToTransmit < minData)
	    	    {
#ifdef SCHEDULER_DEBUG
	      std::cout << "\t selected minData " << std::endl;
#endif
	    		  dataToTransmit = minData;
	    	    }
	    	  else
	    	    {
	    		  if (dataToTransmit < maxData)
	    			{
	    			     dataToTransmit=maxData;
#ifdef SCHEDULER_DEBUG
	      std::cout << "\t selected maxData " << std::endl;
#endif
	    			}
	    	    }
	        }


#ifdef SCHEDULER_DEBUG
	      std::cout << "\t selected flow " << (*it)->GetApplication ()->GetApplicationID () << std::endl;
	      qos->Print ();
	      std::cout << "\t queue size  " <<  queueSize << "\n\t DATA TO TRANSMIT " << (ceil)(dataToTransmit) << std::endl;
#endif

	      qos->UpdateQ (queueSize);
	      qos->UpdateU ((ceil)(dataToTransmit));
	      qos->SetDataToTransmit ((ceil)(dataToTransmit+80));
		}
	}
}


void
DL_TLS_PacketScheduler::UpdateDataToTransmitAndAverageDataRate (void)
{
	RrcEntity *rrc = GetMacEntity ()->GetDevice ()->GetProtocolStack ()->GetRrcEntity ();
	RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer ();

	for (std::vector<RadioBearer* >::iterator it = bearers->begin (); it != bearers->end (); it++)
	{
		RadioBearer *bearer = (*it);

	  // UPDATE DATA TO TRASMIT FOR RT FLOWS
		if (bearer->GetApplication ()->GetApplicationType () == Application::APPLICATION_TYPE_TRACE_BASED
			||
			bearer->GetApplication ()->GetApplicationType () == Application::APPLICATION_TYPE_VOIP)
		{
			QoSForFLS *qos = (QoSForFLS*) bearer->GetQoSParameters ();
			int dataToTransmit = qos->GetDataToTransmit ();
			int transmittedData = bearer->GetTransmittedBytes ();
			if (transmittedData >= dataToTransmit)
			{
				qos->SetDataToTransmit (0);
			}
			else
			{
				qos->SetDataToTransmit (dataToTransmit - transmittedData);
			}
		}

	  // UPDATE AVERAGE TRANSMISSION RATE
		bearer->UpdateAverageTransmissionRate ();
	}
}
