/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Technische Universität Berlin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Piotr Gawlowicz <gawlowicz@tkn.tu-berlin.de>
 */

#include "tcp-rl-env.h"
#include "ns3/tcp-header.h"
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/tcp-socket-base.h"
#include <vector>
#include <numeric>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ns3::TcpGymEnv");
NS_OBJECT_ENSURE_REGISTERED (TcpGymEnv);

TcpGymEnv::TcpGymEnv(std::string prot)
{
  NS_LOG_FUNCTION (this);
  SetOpenGymInterface(OpenGymInterface::Get());

  // create protocol
  // NS_LOG_UNCOND("creating protocol: " << prot);
  // NS_LOG_UNCOND("rl?: " << (prot.find("Rl")));
  if (prot.find("Rl") == std::string::npos)
  {
    ObjectFactory factory;
    factory.SetTypeId(TypeId::LookupByName(prot));
    Ptr<Object> obj = factory.Create();
    m_prot = obj->GetObject<TcpCongestionOps>();
    NS_LOG_UNCOND("protocol created: " << prot);
  } else
  {
    m_prot = nullptr;
    NS_LOG_UNCOND("not creating protocol");
  }
  
}

TcpGymEnv::~TcpGymEnv ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
TcpGymEnv::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpGymEnv")
    .SetParent<OpenGymEnv> ()
    .SetGroupName ("OpenGym")
  ;

  return tid;
}

void
TcpGymEnv::DoDispose ()
{
  NS_LOG_FUNCTION (this);
}

void
TcpGymEnv::SetNodeId(uint32_t id)
{
  NS_LOG_FUNCTION (this);
  m_nodeId = id;
}

void
TcpGymEnv::SetSocketUuid(uint32_t id)
{
  NS_LOG_FUNCTION (this);
  m_socketUuid = id;
}

std::string
TcpGymEnv::GetTcpCongStateName(const TcpSocketState::TcpCongState_t state)
{
  std::string stateName = "UNKNOWN";
  switch(state) {
    case TcpSocketState::CA_OPEN:
      stateName = "CA_OPEN";
      break;
    case TcpSocketState::CA_DISORDER:
      stateName = "CA_DISORDER";
      break;
    case TcpSocketState::CA_CWR:
      stateName = "CA_CWR";
      break;
    case TcpSocketState::CA_RECOVERY:
      stateName = "CA_RECOVERY";
      break;
    case TcpSocketState::CA_LOSS:
      stateName = "CA_LOSS";
      break;
    case TcpSocketState::CA_LAST_STATE:
      stateName = "CA_LAST_STATE";
      break;
    default:
       stateName = "UNKNOWN";
       break;
  }
  return stateName;
}

std::string
TcpGymEnv::GetTcpCAEventName(const TcpSocketState::TcpCAEvent_t event)
{
  std::string eventName = "UNKNOWN";
  switch(event) {
    case TcpSocketState::CA_EVENT_TX_START:
      eventName = "CA_EVENT_TX_START";
      break;
    case TcpSocketState::CA_EVENT_CWND_RESTART:
      eventName = "CA_EVENT_CWND_RESTART";
      break;
    case TcpSocketState::CA_EVENT_COMPLETE_CWR:
      eventName = "CA_EVENT_COMPLETE_CWR";
      break;
    case TcpSocketState::CA_EVENT_LOSS:
      eventName = "CA_EVENT_LOSS";
      break;
    case TcpSocketState::CA_EVENT_ECN_NO_CE:
      eventName = "CA_EVENT_ECN_NO_CE";
      break;
    case TcpSocketState::CA_EVENT_ECN_IS_CE:
      eventName = "CA_EVENT_ECN_IS_CE";
      break;
    case TcpSocketState::CA_EVENT_DELAYED_ACK:
      eventName = "CA_EVENT_DELAYED_ACK";
      break;
    case TcpSocketState::CA_EVENT_NON_DELAYED_ACK:
      eventName = "CA_EVENT_NON_DELAYED_ACK";
      break;
    default:
       eventName = "UNKNOWN";
       break;
  }
  return eventName;
}

/*
Define action space
*/
Ptr<OpenGymSpace>
TcpGymEnv::GetActionSpace()
{
  uint32_t parameterNum = 2;
  float low = 0.0;
  float high = 1000000000.0;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<uint32_t> ();

  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_INFO ("MyGetActionSpace: " << box);
  return box;
}

/*
Define game over condition
*/
bool
TcpGymEnv::GetGameOver()
{
  m_isGameOver = false;
  bool test = false;
  static float stepCounter = 0.0;
  stepCounter += 1;
  if (stepCounter == 10 && test) {
      m_isGameOver = true;
  }
  NS_LOG_INFO ("MyGetGameOver: " << m_isGameOver);
  return m_isGameOver;
}

/*
Define reward function
*/
float
TcpGymEnv::GetReward()
{
  NS_LOG_INFO("MyGetReward: " << m_envReward);
  return m_envReward;
}

/*
Define extra info. Optional
*/
std::string
TcpGymEnv::GetExtraInfo()
{
  NS_LOG_INFO("MyGetExtraInfo: " << m_info);
  return m_info;
}

/*
Execute received actions
*/
bool
TcpGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
  Ptr<OpenGymBoxContainer<uint32_t> > box = DynamicCast<OpenGymBoxContainer<uint32_t> >(action);
  m_new_cWnd = box->GetValue(0);
  m_new_ssThresh = box->GetValue(1);

  NS_LOG_INFO ("MyExecuteActions: " << action);
  return true;
}

NS_OBJECT_ENSURE_REGISTERED (TcpTimeStepGymEnv);

TcpTimeStepGymEnv::TcpTimeStepGymEnv (std::string prot, Time timeStep) : TcpGymEnv(prot)
{
  NS_LOG_FUNCTION (this);
  m_timeStep = timeStep;
  m_envReward = 0.0;
  m_new_ssThresh = 0;
  m_new_cWnd = 0;
}

void
TcpTimeStepGymEnv::ScheduleNextStateRead ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Schedule (m_timeStep, &TcpTimeStepGymEnv::ScheduleNextStateRead, this);
  Notify();
}

TcpTimeStepGymEnv::~TcpTimeStepGymEnv ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
TcpTimeStepGymEnv::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpTimeStepGymEnv")
    .SetParent<TcpGymEnv> ()
    .SetGroupName ("OpenGym")
    // .AddConstructor<TcpTimeStepGymEnv> ()
    // .AddAttribute ("FlowMonitor",
    //            "A pointer to the FlowMonitor. Default: null",
    //            nullptr,
    //            MakePointerAccessor(m_flowmon),
    //            MakePointerChecker<FlowMonitor>())
  ;

  return tid;
}

void
TcpTimeStepGymEnv::DoDispose ()
{
  NS_LOG_FUNCTION (this);
}

/*
Define observation space
*/
Ptr<OpenGymSpace>
TcpTimeStepGymEnv::GetObservationSpace()
{
  // 0: socket unique ID
  // 1: sim time in us
  // 2: node ID
  // 3: ssThresh
  // 4: cWnd
  // 5: segmentSize
  // 6: bytesTx
  // 7: bytesRx
  // 8: bytesInFlight
  // 9: segmentsAcked
  // 10: rtt
  uint32_t parameterNum = 13;
  float low = 0.0;
  float high = 1000000000.0;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<double> ();

  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_INFO ("MyGetObservationSpace: " << box);
  return box;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer>
TcpTimeStepGymEnv::GetObservation()
{
  uint32_t parameterNum = 13;
  std::vector<uint32_t> shape = {parameterNum,};

  Ptr<OpenGymBoxContainer<double> > box = CreateObject<OpenGymBoxContainer<double> >(shape);

  // 0: socket unique ID
  // 1: sim time in us
  // 2: node ID
  // 3: ssThresh
  // 4: cWnd
  // 5: segmentSize
  // 6: bytesTx
  // 7: bytesRx
  // 8: bytesInFlight
  // 9: segmentsAcked
  // 10: rtt
  box->AddValue(m_socketUuid);
  box->AddValue(Simulator::Now().GetMicroSeconds ());
  box->AddValue(m_nodeId);
  box->AddValue(m_curr_ssthresh);
  box->AddValue(m_curr_cwnd);
  box->AddValue(m_tcb->m_segmentSize);
  box->AddValue(m_bytesTx);
  box->AddValue(m_bytesRx);
  box->AddValue(m_bytesInFlight);
  box->AddValue(m_segmentsAcked);
  box->AddValue(m_rtt.GetMicroSeconds());
  box->AddValue(m_lastAckedSeq);
  box->AddValue(m_nextTxSeq);

  m_envReward = 0;
  
  // Print data
  NS_LOG_INFO ("MyGetObservation: " << box);

  // Reset counters
  m_segmentsAcked = 0;
  m_bytesTx = 0;
  m_bytesRx = 0;

  return box;
}

void
TcpTimeStepGymEnv::TxPktTrace(Ptr<const Packet> packet, const TcpHeader&, Ptr<const TcpSocketBase> socket)
{
  NS_LOG_FUNCTION (this);
  m_bytesTx += packet->GetSize ();
}

void
TcpTimeStepGymEnv::RxPktTrace(Ptr<const Packet> packet, const TcpHeader&, Ptr<const TcpSocketBase>)
{
  NS_LOG_FUNCTION (this);
  m_bytesRx += packet->GetSize ();
}

void TcpTimeStepGymEnv::CwndTrace(uint32_t old_val, uint32_t new_val)
{
  NS_LOG_FUNCTION (this);
  m_curr_cwnd = new_val;
}

void TcpTimeStepGymEnv::BytesInFlightTrace(uint32_t, uint32_t new_val)
{
  NS_LOG_FUNCTION (this);
  m_bytesInFlight = new_val;
}

void TcpTimeStepGymEnv::RttTrace(Time, Time new_val)
{
  NS_LOG_FUNCTION (this);
  m_rtt = new_val;
}


void TcpTimeStepGymEnv::SsThreshTrace(uint32_t, uint32_t new_val)
{
  NS_LOG_FUNCTION (this);
  m_curr_ssthresh = new_val;
}

void TcpTimeStepGymEnv::HighestSequenceTrace(SequenceNumber32, SequenceNumber32 new_val)
{
  NS_LOG_FUNCTION (this);
  m_lastAckedSeq = new_val.GetValue();
}

void TcpTimeStepGymEnv::NextTxSequenceTrace(SequenceNumber32, SequenceNumber32 new_val)
{
  NS_LOG_FUNCTION (this);
  m_nextTxSeq = new_val.GetValue();
}

uint32_t
TcpTimeStepGymEnv::GetSsThresh (Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " GetSsThresh, BytesInFlight: " << bytesInFlight);
  m_tcb = tcb;
  m_bytesInFlight = bytesInFlight;

  if (!m_started) {
    m_started = true;
    Notify();
    ScheduleNextStateRead();
  }

  // action
  if (m_prot != nullptr)
  {
    m_new_ssThresh = m_prot->GetSsThresh(tcb, bytesInFlight);
  }  
  
  return m_new_ssThresh;
}

void
TcpTimeStepGymEnv::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " IncreaseWindow, SegmentsAcked: " << segmentsAcked);
  m_tcb = tcb;
  
  m_bytesInFlight = tcb->m_bytesInFlight;

  if (!m_started)
  {
    m_started = true;
    Notify();
    ScheduleNextStateRead();
  }
  
  // action
  if (m_prot != nullptr)
  {
    m_prot->IncreaseWindow(tcb, segmentsAcked);
    m_new_cWnd = tcb->m_cWnd;
  }
  else
  {
    tcb->m_cWnd = m_new_cWnd;
  }
}

void
TcpTimeStepGymEnv::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " PktsAcked, SegmentsAcked: " << segmentsAcked << " Rtt: " << rtt);
  m_tcb = tcb;
  m_rtt = rtt;
  m_segmentsAcked += segmentsAcked;
  
  if (m_prot != nullptr)
  {
    m_prot->PktsAcked(tcb, segmentsAcked, rtt);
  }
}

void
TcpTimeStepGymEnv::CongestionStateSet (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this);
  std::string stateName = GetTcpCongStateName(newState);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " CongestionStateSet: " << newState << " " << stateName);
  m_tcb = tcb;
  
  if (m_prot != nullptr)
  {
    m_prot->CongestionStateSet(tcb, newState);
  }
}

void
TcpTimeStepGymEnv::CwndEvent (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCAEvent_t event)
{
  NS_LOG_FUNCTION (this);
  std::string eventName = GetTcpCAEventName(event);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " CwndEvent: " << event << " " << eventName);
  m_tcb = tcb;
  
  if (m_prot != nullptr)
  {
    m_prot->CwndEvent(tcb, event);
  }
}

void TcpTimeStepGymEnv::CongControl(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateConnection &rc, const TcpRateOps::TcpRateSample &rs)
{
  NS_LOG_FUNCTION (this);
  m_tcb = tcb;
  
  if (m_prot != nullptr)
  {
    if (m_prot->HasCongControl())
    {
      m_prot->CongControl(tcb, rc, rs);
    }
  }
  else
  {
    tcb->m_cWnd = m_new_cWnd;
  }
}

} // namespace ns3
