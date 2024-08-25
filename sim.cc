/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Thang Bui-Nguyen
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
 * Author: Thang Bui-Nguyen
 * Based on script: ./examples/tcp/tcp-variants-comparison.cc
 *
 * Topology:
 *
 *   Right Leafs (Clients)                      Left Leafs (Sinks)
 *           |            \                    /        |
 *           |             \    bottleneck    /         |
 *           |              R0--------------R1          |
 *           |             /                  \         |
 *           |   access   /                    \ access |
 *           N -----------                      --------N
 */

#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/applications-module.h"
#include "ns3/error-model.h"
#include "ns3/tcp-header.h"
#include "ns3/enum.h"
#include "ns3/event-id.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/traffic-control-module.h"

#include "ns3/opengym-module.h"
#include "tcp-rl.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TcpVariantsComparison");

// static std::map<uint32_t, Ptr<OutputStreamWrapper>> cWndStream;
// static std::map<uint32_t, Ptr<OutputStreamWrapper>> rttStream;

// static std::vector<uint32_t> rxPkts;

// static void
// CountRxPkts(uint32_t sinkId, Ptr<const Packet> packet, const Address & srcAddr)
// {
//   rxPkts[sinkId]++;
// }

// static void
// PrintRxCount()
// {
//   uint32_t size = rxPkts.size();
//   NS_LOG_UNCOND("RxPkts:");
//   for (uint32_t i=0; i<size; i++){
//     NS_LOG_UNCOND("---SinkId: "<< i << " RxPkts: " << rxPkts.at(i));
//   }
// }

// static uint32_t
// GetNodeIdFromContext(std::string context)
// {
//     const std::size_t n1 = context.find_first_of('/', 1);
//     const std::size_t n2 = context.find_first_of('/', n1 + 1);
//     return std::stoul(context.substr(n1 + 1, n2 - n1 - 1));
// }

// static void
// CwndTracer(std::string context, uint32_t oldval, uint32_t newval)
// {
//     uint32_t nodeId = GetNodeIdFromContext(context);

//     // if (firstCwnd[nodeId])
//     // {
//     //     *cWndStream[nodeId]->GetStream() << "0.0 " << oldval << std::endl;
//     //     firstCwnd[nodeId] = false;
//     // }
//     *cWndStream[nodeId]->GetStream() << Simulator::Now().GetSeconds() << " " << newval << std::endl;
//     // NS_LOG_UNCOND("time: " << Simulator::Now().GetSeconds() << " cwnd: " << newval);
//     // cWndValue[nodeId] = newval;

//     // if (!firstSshThr[nodeId])
//     // {
//     //     *ssThreshStream[nodeId]->GetStream()
//     //         << Simulator::Now().GetSeconds() << " " << ssThreshValue[nodeId] << std::endl;
//     // }
// }

// static void
// RttTracer(std::string context, Time oldval, Time newval)
// {
//     uint32_t nodeId = GetNodeIdFromContext(context);

//     // if (firstRtt[nodeId])
//     // {
//     //     *rttStream[nodeId]->GetStream() << "0.0 " << oldval.GetSeconds() << std::endl;
//     //     firstRtt[nodeId] = false;
//     // }

//     // NS_LOG_UNCOND("time: " << Simulator::Now().GetSeconds() << " rtt: " << newval.GetSeconds());
  
//     *rttStream[nodeId]->GetStream()
//         << Simulator::Now().GetSeconds() << " " << newval.GetSeconds() << std::endl;
// }

// static void
// TraceCwnd(std::string cwnd_tr_file_name, uint32_t nodeId)
// {
//     AsciiTraceHelper ascii;
//     cWndStream[nodeId] = ascii.CreateFileStream(cwnd_tr_file_name);
//     Config::Connect("/NodeList/" + std::to_string(nodeId) +
//                         "/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow",
//                     MakeCallback(&CwndTracer));
// }

// static void
// TraceRtt(std::string rtt_tr_file_name, uint32_t nodeId)
// {
//     AsciiTraceHelper ascii;
//     rttStream[nodeId] = ascii.CreateFileStream(rtt_tr_file_name);
//     Config::Connect("/NodeList/" + std::to_string(nodeId) + "/$ns3::TcpL4Protocol/SocketList/0/RTT",
//                     MakeCallback(&RttTracer));
// }

int main (int argc, char *argv[])
{
  uint32_t openGymPort = 5555;
  double tcpEnvTimeStep = 0.1;

  uint32_t num_flows = 1;
  std::string transport_prot = "TcpBic";
  std::string rl_env = "TcpRl";
  double error_p = 0.0;
  std::string bottleneck_bandwidth = "2Mbps";
  std::string bottleneck_delay = "0.01ms";
  std::string access_bandwidth = "10Mbps";
  std::string access_delay = "20ms";
  std::string prefix_file_name = "/home/thang/ns-allinone-3.42/ns-3.42/contrib/opengym/examples/qsiurp/TcpVariantsComparison";
  uint64_t data_mbytes = 0;
  uint32_t mtu_bytes = 400;
  double duration = 10.0;
  uint32_t run = 0;
  bool flow_monitor = false;
  bool sack = true;
  std::string queue_disc_type = "ns3::PfifoFastQueueDisc";
  std::string recovery = "ns3::TcpClassicRecovery";

  CommandLine cmd;
  // required parameters for OpenGym interface
  cmd.AddValue ("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
  cmd.AddValue ("simSeed", "Seed for random generator. Default: 1", run);
  cmd.AddValue ("envTimeStep", "Time step interval for time-based TCP env [s]. Default: 0.1s", tcpEnvTimeStep);
  // other parameters
  cmd.AddValue("num_flows", "Number of flows", num_flows);
  cmd.AddValue ("rl_env", "RL env to use: TcpRl, TcpRlTimeBased", rl_env);
  cmd.AddValue ("transport_prot", "Transport protocol to use: TcpNewReno, "
                "TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas, TcpScalable, TcpVeno, "
                "TcpBic, TcpYeah, TcpIllinois, TcpWestwoodPlus, TcpLedbat, "
		            "TcpLp, TcpRl, TcpRlTimeBased", transport_prot);
  cmd.AddValue ("error_p", "Packet error rate", error_p);
  cmd.AddValue ("bottleneck_bandwidth", "Bottleneck bandwidth", bottleneck_bandwidth);
  cmd.AddValue ("bottleneck_delay", "Bottleneck delay", bottleneck_delay);
  cmd.AddValue ("access_bandwidth", "Access link bandwidth", access_bandwidth);
  cmd.AddValue ("access_delay", "Access link delay", access_delay);
  // cmd.AddValue ("prefix_name", "Prefix of output trace file", prefix_file_name);
  cmd.AddValue ("data", "Number of Megabytes of data to transmit", data_mbytes);
  cmd.AddValue ("mtu", "Size of IP packets to send in bytes", mtu_bytes);
  cmd.AddValue ("duration", "Time to allow flows to run in seconds", duration);
  cmd.AddValue ("run", "Run index (for setting repeatable seeds)", run);
  cmd.AddValue ("flow_monitor", "Enable flow monitor", flow_monitor);
  cmd.AddValue ("queue_disc_type", "Queue disc type for gateway (e.g. ns3::CoDelQueueDisc)", queue_disc_type);
  cmd.AddValue ("sack", "Enable or disable SACK option", sack);
  cmd.AddValue ("recovery", "Recovery algorithm type to use (e.g., ns3::TcpPrrRecovery", recovery);
  cmd.Parse (argc, argv);

  transport_prot = std::string ("ns3::") + transport_prot;
  rl_env = std::string("ns3::") + rl_env;

  SeedManager::SetSeed (1);
  SeedManager::SetRun (run);

  NS_LOG_UNCOND("Ns3Env parameters:");
  // if (transport_prot.compare ("ns3::TcpRl") == 0 or transport_prot.compare ("ns3::TcpRlTimeBased") == 0)
  // {
  NS_LOG_UNCOND("--openGymPort: " << openGymPort);
  // } else {
  //   NS_LOG_UNCOND("--openGymPort: No OpenGym");
  // }

  NS_LOG_UNCOND("--seed: " << run);
  NS_LOG_UNCOND("--Tcp version: " << transport_prot);
  NS_LOG_UNCOND("--RL env: " << rl_env);
  NS_LOG_UNCOND("--duration: " << duration);
  NS_LOG_UNCOND("--to transmit: " << data_mbytes);

  // Create OpenGym env
  Ptr<OpenGymInterface> openGymInterface = OpenGymInterface::Get(openGymPort);

  // Calculate the ADU size
  Header* temp_header = new Ipv4Header();
  uint32_t ip_header = temp_header->GetSerializedSize();
  NS_LOG_LOGIC("IP Header size is: " << ip_header);
  delete temp_header;
  temp_header = new TcpHeader();
  uint32_t tcp_header = temp_header->GetSerializedSize();
  NS_LOG_LOGIC("TCP Header size is: " << tcp_header);
  delete temp_header;
  uint32_t tcp_adu_size = mtu_bytes - 20 - (ip_header + tcp_header); // 20???
  NS_LOG_LOGIC("TCP ADU size is: " << tcp_adu_size);

  // 2 MB of TCP buffer
  Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(1 << 21));
  Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(1 << 21));
  Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(sack));

  Config::SetDefault("ns3::TcpL4Protocol::RecoveryType",
                     TypeIdValue(TypeId::LookupByName(recovery)));

  // Set the simulation start and stop time
  double start_time = 0.1;
  double stop_time = start_time + duration;

  // Select TCP variant
  TypeId tcpTid;
  // NS_ABORT_MSG_UNLESS(TypeId::LookupByNameFailSafe(transport_prot, &tcpTid),
  //                     "TypeId " << transport_prot << " not found");
  Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                     TypeIdValue(TypeId::LookupByName(rl_env)));
  // Config::SetDefault("ns3::TcpRl::Protocol",
  //                    StringValue(transport_prot));
  Config::SetDefault("ns3::TcpRlTimeBased::Protocol",
                     StringValue(transport_prot));
  Config::SetDefault("ns3::TcpRlTimeBased::TimeStep",
                     TimeValue(Seconds(tcpEnvTimeStep)));

  // Create gateways, sources, and sinks
  NodeContainer gateways;
  gateways.Create(1);
  NodeContainer sources;
  sources.Create(num_flows);
  NodeContainer sinks;
  sinks.Create(num_flows);

  // Configure the error model
  // Here we use RateErrorModel with packet error rate
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
  uv->SetStream(50); // set seed
  RateErrorModel error_model;
  error_model.SetRandomVariable(uv);
  error_model.SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
  error_model.SetRate(error_p);

  PointToPointHelper UnReLink;
  UnReLink.SetDeviceAttribute("DataRate", StringValue(bottleneck_bandwidth));
  UnReLink.SetChannelAttribute("Delay", StringValue(bottleneck_delay));
  UnReLink.SetDeviceAttribute("ReceiveErrorModel", PointerValue(&error_model));

  InternetStackHelper stack;
  stack.InstallAll();

  TrafficControlHelper tchPfifo;
  tchPfifo.SetRootQueueDisc("ns3::PfifoFastQueueDisc");

  TrafficControlHelper tchCoDel;
  tchCoDel.SetRootQueueDisc("ns3::CoDelQueueDisc");

  Ipv4AddressHelper address;
  address.SetBase("10.0.0.0", "255.255.255.0");

  // Configure the sources and sinks net devices
  // and the channels between the sources/sinks and the gateways
  PointToPointHelper LocalLink;
  LocalLink.SetDeviceAttribute("DataRate", StringValue(access_bandwidth));
  LocalLink.SetChannelAttribute("Delay", StringValue(access_delay));

  Ipv4InterfaceContainer sink_interfaces;

  DataRate access_b(access_bandwidth);
  DataRate bottle_b(bottleneck_bandwidth);
  Time access_d(access_delay);
  Time bottle_d(bottleneck_delay);

  uint32_t size = static_cast<uint32_t>((std::min(access_b, bottle_b).GetBitRate() / 8) *
                                        ((access_d + bottle_d) * 2).GetSeconds());

  Config::SetDefault("ns3::PfifoFastQueueDisc::MaxSize",
                     QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, size / mtu_bytes)));
  Config::SetDefault("ns3::CoDelQueueDisc::MaxSize",
                     QueueSizeValue(QueueSize(QueueSizeUnit::BYTES, size)));

  for (uint32_t i = 0; i < num_flows; i++)
  {
    NetDeviceContainer devices;
    devices = LocalLink.Install(sources.Get(i), gateways.Get(0));
    tchPfifo.Install(devices);
    address.NewNetwork();
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    devices = UnReLink.Install(gateways.Get(0), sinks.Get(i));
    if (queue_disc_type == "ns3::PfifoFastQueueDisc")
    {
        tchPfifo.Install(devices);
    }
    else if (queue_disc_type == "ns3::CoDelQueueDisc")
    {
        tchCoDel.Install(devices);
    }
    else
    {
        NS_FATAL_ERROR("Queue not recognized. Allowed values are ns3::CoDelQueueDisc or "
                       "ns3::PfifoFastQueueDisc");
    }
    address.NewNetwork();
    interfaces = address.Assign(devices);
    sink_interfaces.Add(interfaces.Get(1));
  }

  NS_LOG_INFO("Initialize Global Routing.");
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  uint16_t port = 50000;
  Address sinkLocalAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
  PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", sinkLocalAddress);

  for (uint32_t i = 0; i < sources.GetN(); i++)
  {
    AddressValue remoteAddress(InetSocketAddress(sink_interfaces.GetAddress(i, 0), port));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(tcp_adu_size));
    BulkSendHelper ftp("ns3::TcpSocketFactory", Address());
    ftp.SetAttribute("Remote", remoteAddress);
    ftp.SetAttribute("SendSize", UintegerValue(tcp_adu_size));
    ftp.SetAttribute("MaxBytes", UintegerValue(data_mbytes * 1000000));

    ApplicationContainer sourceApp = ftp.Install(sources.Get(i));
    sourceApp.Start(Seconds(start_time * i));
    sourceApp.Stop(Seconds(stop_time - 3));

    sinkHelper.SetAttribute("Protocol", TypeIdValue(TcpSocketFactory::GetTypeId()));
    ApplicationContainer sinkApp = sinkHelper.Install(sinks.Get(i));
    sinkApp.Start(Seconds(start_time * i));
    sinkApp.Stop(Seconds(stop_time));
  }

  Simulator::Stop(Seconds(stop_time));
  Simulator::Run();
  
  Simulator::Destroy();

  openGymInterface->NotifySimulationEnd();

  return 0;
}
