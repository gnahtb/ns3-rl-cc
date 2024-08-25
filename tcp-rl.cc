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

#include "tcp-rl.h"
#include "tcp-rl-env.h"
#include "ns3/tcp-header.h"
#include "ns3/object.h"
#include "ns3/node-list.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/tcp-socket-base.h"
#include "ns3/tcp-l4-protocol.h"


namespace ns3 {


NS_OBJECT_ENSURE_REGISTERED (TcpSocketDerived);

TypeId
TcpSocketDerived::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpSocketDerived")
    .SetParent<TcpSocketBase> ()
    .SetGroupName ("Internet")
    .AddConstructor<TcpSocketDerived> ()
  ;
  return tid;
}

TypeId
TcpSocketDerived::GetInstanceTypeId () const
{
  return TcpSocketDerived::GetTypeId ();
}

TcpSocketDerived::TcpSocketDerived (void)
{
}

Ptr<TcpCongestionOps>
TcpSocketDerived::GetCongestionControlAlgorithm ()
{
  return m_congestionControl;
}

TcpSocketDerived::~TcpSocketDerived (void)
{
}


NS_LOG_COMPONENT_DEFINE ("ns3::TcpRlBase");
NS_OBJECT_ENSURE_REGISTERED (TcpRlBase);

TypeId
TcpRlBase::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpRlBase")
    .SetParent<TcpCongestionOps> ()
    .SetGroupName ("Internet")
    .AddConstructor<TcpRlBase> ()
  ;
  return tid;
}

TcpRlBase::TcpRlBase (void)
  : TcpCongestionOps ()
{
  NS_LOG_FUNCTION (this);
  m_tcpSocket = 0;
  m_tcpGymEnv = 0;
}

TcpRlBase::TcpRlBase (const TcpRlBase& sock)
  : TcpCongestionOps (sock)
{
  NS_LOG_FUNCTION (this);
  m_tcpSocket = 0;
  m_tcpGymEnv = 0;
}

TcpRlBase::~TcpRlBase (void)
{
  m_tcpSocket = 0;
  m_tcpGymEnv = 0;
}

uint64_t
TcpRlBase::GenerateUuid ()
{
  static uint64_t uuid = 0;
  uuid++;
  return uuid;
}

void
TcpRlBase::CreateGymEnv()
{
  NS_LOG_FUNCTION (this);
  // should never be called, only child classes: TcpRl and TcpRlTimeBased
}

void
TcpRlBase::ConnectSocketCallbacks()
{
  NS_LOG_FUNCTION (this);

  bool foundSocket = false;
  Ptr<TcpSocketBase> rxTcpSocket = nullptr;
  int nodeId = -1;
  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) {
    Ptr<Node> node = *i;
    Ptr<TcpL4Protocol> tcp = node->GetObject<TcpL4Protocol> ();

    ObjectVectorValue socketVec;
    tcp->GetAttribute ("SocketList", socketVec);
    NS_LOG_UNCOND("Node: " << node->GetId() << " TCP socket num: " << socketVec.GetN());

    uint32_t sockNum = socketVec.GetN();
    for (uint32_t j=0; j<sockNum; j++) {
      Ptr<Object> sockObj = socketVec.Get(j);
      Ptr<TcpSocketBase> tcpSocket = DynamicCast<TcpSocketBase> (sockObj);
      NS_LOG_INFO("Node: " << node->GetId() << " TCP Socket: " << tcpSocket);
      if(!tcpSocket) { continue; }

      Ptr<TcpSocketDerived> dtcpSocket = StaticCast<TcpSocketDerived>(tcpSocket);
      Ptr<TcpCongestionOps> ca = dtcpSocket->GetCongestionControlAlgorithm();
      NS_LOG_INFO("CA name: " << ca->GetName());
      Ptr<TcpRlBase> rlCa = DynamicCast<TcpRlBase>(ca);
      if (rlCa == this) {
        NS_LOG_UNCOND("Found TcpRl CA!");
        foundSocket = true;
        m_tcpSocket = tcpSocket;

        int rxNodeId = node->GetId() + 1; // TODO: replace 1 with num_flows
        Ptr<Node> rxNode = NodeList::GetNode(rxNodeId);
        Ptr<TcpL4Protocol> rxTcp = rxNode->GetObject<TcpL4Protocol> ();
        ObjectVectorValue rxSocketVec;
        rxTcp->GetAttribute ("SocketList", rxSocketVec);
        NS_LOG_INFO("RX Node: " << rxNode->GetId() << " RX TCP socket num: " << rxSocketVec.GetN());
        Ptr<Object> rxSockObj = rxSocketVec.Get(1);
        rxTcpSocket = DynamicCast<TcpSocketBase> (rxSockObj);
        
        break;
      }
    }

    if (foundSocket) {
      nodeId = node->GetId();
      break;
    }
  }

  NS_ASSERT_MSG(m_tcpSocket, "TCP socket was not found.");

  if(m_tcpSocket)
  {
    NS_LOG_INFO("Found TCP Socket: " << m_tcpSocket);
    m_tcpSocket->TraceConnectWithoutContext ("Tx", MakeCallback (&TcpGymEnv::TxPktTrace, m_tcpGymEnv));
    // m_tcpSocket->TraceConnectWithoutContext ("Rx", MakeCallback (&TcpGymEnv::RxPktTrace, m_tcpGymEnv));
    // m_tcpSocket->TraceConnectWithoutContext ("CongestionWindow", MakeCallback (&TcpGymEnv::CwndTrace, m_tcpGymEnv));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeId) + "/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&TcpGymEnv::CwndTrace, m_tcpGymEnv));
    m_tcpSocket->TraceConnectWithoutContext ("BytesInFlight", MakeCallback (&TcpGymEnv::BytesInFlightTrace, m_tcpGymEnv));
    m_tcpSocket->TraceConnectWithoutContext ("RTT", MakeCallback (&TcpGymEnv::RttTrace, m_tcpGymEnv));
    // Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeId) + "/$ns3::TcpL4Protocol/SocketList/0/RTT", MakeCallback (&TcpGymEnv::RttTrace, m_tcpGymEnv));
    m_tcpSocket->TraceConnectWithoutContext ("SlowStartThreshold", MakeCallback (&TcpGymEnv::SsThreshTrace, m_tcpGymEnv));

    rxTcpSocket->TraceConnectWithoutContext ("Rx", MakeCallback (&TcpGymEnv::RxPktTrace, m_tcpGymEnv));
    
    NS_LOG_UNCOND("Connect socket callbacks " << m_tcpSocket->GetNode()->GetId());
    
    m_tcpGymEnv->SetNodeId(m_tcpSocket->GetNode()->GetId());
  }
}

std::string
TcpRlBase::GetName () const
{
  return "TcpRlBase";
}

void TcpRlBase::CongControl(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateConnection &rc, const TcpRateOps::TcpRateSample &rs)
{
  NS_LOG_FUNCTION (this);
  if (!m_tcpGymEnv) {
    CreateGymEnv();
  }
  if (m_tcpGymEnv) {
    m_tcpGymEnv->CongControl(tcb, rc, rs);
  }
}   

uint32_t
TcpRlBase::GetSsThresh (Ptr<const TcpSocketState> state,
                         uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this << state << bytesInFlight);

  if (!m_tcpGymEnv) {
    CreateGymEnv();
  }

  uint32_t newSsThresh = 0;
  if (m_tcpGymEnv) {
      newSsThresh = m_tcpGymEnv->GetSsThresh(state, bytesInFlight);
  }

  return newSsThresh;
}

void
TcpRlBase::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (!m_tcpGymEnv) {
    CreateGymEnv();
  }

  if (m_tcpGymEnv) {
     m_tcpGymEnv->IncreaseWindow(tcb, segmentsAcked);
  }
}

void
TcpRlBase::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt)
{
  NS_LOG_FUNCTION (this);

  if (!m_tcpGymEnv) {
    CreateGymEnv();
  }

  if (m_tcpGymEnv) {
     m_tcpGymEnv->PktsAcked(tcb, segmentsAcked, rtt);
  }
}

void
TcpRlBase::CongestionStateSet (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this);

  if (!m_tcpGymEnv) {
    CreateGymEnv();
  }

  if (m_tcpGymEnv) {
     m_tcpGymEnv->CongestionStateSet(tcb, newState);
  }
}

void
TcpRlBase::CwndEvent (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCAEvent_t event)
{
  NS_LOG_FUNCTION (this);

  if (!m_tcpGymEnv) {
    CreateGymEnv();
  }

  if (m_tcpGymEnv) {
     m_tcpGymEnv->CwndEvent(tcb, event);
  }
}

Ptr<TcpCongestionOps>
TcpRlBase::Fork ()
{
  return CopyObject<TcpRlBase> (this);
}


bool TcpRlBase::HasCongControl()
{
  return true;
}

NS_OBJECT_ENSURE_REGISTERED (TcpRlTimeBased);

TypeId
TcpRlTimeBased::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpRlTimeBased")
    .SetParent<TcpRlBase> ()
    .SetGroupName ("Internet")
    .AddConstructor<TcpRlTimeBased> ()
    .AddAttribute ("Protocol", "Underlying protocol.",
                   StringValue ("ns3::TcpBic"),
                   MakeStringAccessor(&TcpRlTimeBased::m_prot),
                   MakeStringChecker())
    .AddAttribute ("TimeStep", "Time step.",
                   TimeValue (MilliSeconds (100)),
                   MakeTimeAccessor(&TcpRlTimeBased::m_timeStep),
                   MakeTimeChecker())
  ;
  return tid;
}

TcpRlTimeBased::TcpRlTimeBased (void) : TcpRlBase ()
{
  NS_LOG_FUNCTION (this);
}

TcpRlTimeBased::TcpRlTimeBased (const TcpRlTimeBased& sock)
  : TcpRlBase (sock)
{
  NS_LOG_FUNCTION (this);
}

TcpRlTimeBased::~TcpRlTimeBased (void)
{
}

std::string
TcpRlTimeBased::GetName () const
{
  return "TcpRlTimeBased";
}

void
TcpRlTimeBased::CreateGymEnv()
{
  NS_LOG_FUNCTION (this);
  Ptr<TcpTimeStepGymEnv> env = CreateObject<TcpTimeStepGymEnv>(m_prot, m_timeStep);
  env->SetSocketUuid(TcpRlBase::GenerateUuid());
  m_tcpGymEnv = env;

  ConnectSocketCallbacks();
}

} // namespace ns3
