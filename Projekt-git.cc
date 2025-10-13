/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 SEBASTIEN DERONNE
 * Copyright (c) 2024 AGH University of Krakow
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
 * Author: Szymon Szott <szott@agh.edu.pl>
 * Based on he-wifi-network.cc by S. Deronne <sebastien.deronne@gmail.com>
 * Last update: 2024-01-30 12:29
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/yans-wifi-channel.h"
#include <chrono>
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/wifi-co-trace-helper.h"
#include "ns3/timestamp-tag.h"
#include <cmath>
#include <vector>
#include "ns3/wifi-phy-rx-trace-helper.h"
#include "ns3/node-list.h"
#include "ns3/timestamp-tag.h"
#include <queue>
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-address.h"
#include <iomanip>
#include <limits>
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/wifi-phy-state.h"
#include <string>
#include "ns3/wifi-mac-header.h"
#include <set>
#include <map>
#include <sstream>
#include "ns3/wifi-phy.h"
#include "ns3/udp-socket-factory.h"



using namespace ns3;

std::queue<Time> txTimes;
Time totalDelay = Seconds(0);

std::map<uint32_t, uint64_t> bytesReceivedPerNode;

template <typename T>
void IncrementCounter(std::map<Mac48Address, T>& counter, Mac48Address address) {
  if (counter.find(address) == counter.end()) {
    counter[address] = 1;
  } else {
    counter[address]++;
  }
}

template <typename T>
T GetCount(const std::map<Mac48Address, T>& counter, Mac48Address address) {
  auto it = counter.find(address);
  if (it != counter.end()) {
    return it->second;
  }
  return 0;
}

std::map<Mac48Address, uint64_t> blockAckRx;
std::map<Mac48Address, uint64_t> blockAckReqRx;
std::map<Mac48Address, uint64_t> psduSucceeded;
std::map<Mac48Address, uint64_t> psduFailed;
std::map<Mac48Address, uint64_t> phyHeaderFailed;
std::map<Mac48Address, uint64_t> rxEventWhileDecodingPreamble;
std::map<Mac48Address, uint64_t> rxEventAbortedByTx;
std::map<Mac48Address, Time> timeFirstReceived;
std::map<Mac48Address, Time> timeLastReceived;

std::map<Ipv4Address, uint32_t> ipToNodeId;

std::map<int, uint64_t> rxDropReasonCounts;

std::map<std::pair<uint32_t,uint32_t>, std::set<std::string>> uniqueFrameKeys;
std::map<std::pair<uint32_t,uint32_t>, double> txTimePerDev, rxTimePerDev, ccaTimePerDev;

static std::map<std::pair<uint32_t,uint32_t>, uint64_t> macTxAttempts;
static std::map<std::pair<uint32_t,uint32_t>, uint64_t> macTxRetries;
static std::map<std::pair<uint32_t,uint32_t>, uint64_t> macTxDrops;
static std::pair<uint32_t,uint32_t> ParseNodeDevFromContext(const std::string& ctx);

static void PhyStateLoggerCtx(std::string context, ns3::Time /*start*/, ns3::Time duration, ns3::WifiPhyState state)
{
  auto key = ParseNodeDevFromContext(context);
  const double d = duration.GetSeconds();
  switch (state)
  {
    case ns3::WifiPhyState::TX:       txTimePerDev[key]  += d; break;
    case ns3::WifiPhyState::RX:       rxTimePerDev[key]  += d; break;
    case ns3::WifiPhyState::CCA_BUSY: ccaTimePerDev[key] += d; break;
    default: break;
  }
}

static std::pair<uint32_t,uint32_t> ParseNodeDevFromContext(const std::string& ctx)
{
  size_t nodeStart = ctx.find("/NodeList/") + 10;
  size_t nodeEnd   = ctx.find("/DeviceList/", nodeStart);
  uint32_t nodeId  = std::stoi(ctx.substr(nodeStart, nodeEnd - nodeStart));

  size_t devStart  = nodeEnd + std::string("/DeviceList/").size();
  size_t devEnd    = ctx.find("/", devStart);
  uint32_t devId   = std::stoi(ctx.substr(devStart, devEnd - devStart));
  return {nodeId, devId};
}

static void MacTxCb(std::string context, Ptr<const ns3::Packet> p)
{
  auto key = ParseNodeDevFromContext(context);

  macTxAttempts[key]++;

  if (p)
  {
    ns3::WifiMacHeader hdr;
    Ptr<ns3::Packet> copy = p->Copy();
    if (copy->PeekHeader(hdr))
    {
      if (hdr.IsRetry())
      {
        macTxRetries[key]++;
      }
    }
  }
}

static void MacTxDropCb(std::string context, Ptr<const ns3::Packet> /*p*/)
{
  auto key = ParseNodeDevFromContext(context);
  macTxDrops[key]++;
}


NS_LOG_COMPONENT_DEFINE ("ex1");

uint64_t totalPacketsSent = 0;
uint64_t totalPacketsReceived = 0;
uint64_t txBytes = 0;
uint64_t rxBytes = 0;
uint32_t rxWhileDecodingCount = 0;
uint64_t preambleGlobal = 0;
uint64_t abortedByTxGlobal = 0;

uint16_t port = 9;

bool    hasPrevArrival = false;
Time    prevArrival    = Seconds(0);

uint64_t iatCount = 0;
double   iatMean  = 0.0;
double   iatM2    = 0.0;

void CountTx(Ptr<const Packet> packet)
{
  totalPacketsSent++;
  txTimes.push(Simulator::Now());
}

static uint64_t rxTagged = 0, rxUntagged = 0;

void CountRx(Ptr<const Packet> packet, const Address &address)
{
  Ipv4Address src = InetSocketAddress::ConvertFrom(address).GetIpv4();
  auto it = ipToNodeId.find(src);
  if (it != ipToNodeId.end()) {
    uint32_t nodeId = it->second;
    bytesReceivedPerNode[nodeId] += packet->GetSize();
  }

  totalPacketsReceived++;

  rxBytes += packet->GetSize();

  Time now = Simulator::Now();
  if (!hasPrevArrival)
  {
    hasPrevArrival = true;
    prevArrival = now;
  }
  else
  {
    Time iat = now - prevArrival;
    prevArrival = now;

    iatCount++;
    double x = iat.GetSeconds();
    double delta = x - iatMean;
    iatMean += delta / static_cast<double>(iatCount);
    iatM2   += delta * (x - iatMean);
  }

  if (!txTimes.empty())
  {
    Time txTime = txTimes.front();
    txTimes.pop();
    Time delay = now - txTime;
    totalDelay += delay;
  }

  TimestampTag ts;
  if (packet->PeekPacketTag(ts))
  {
    rxTagged++;
  }
  else
  {
    rxUntagged++;
  }
}

static double gPhyIdleSec = 0.0, gPhyCcaSec = 0.0, gPhyTxSec = 0.0, gPhyRxSec = 0.0, gPhyOtherSec = 0.0;

static void PhyStateLogger(ns3::Time start, ns3::Time duration, ns3::WifiPhyState state)
{
  const double d = duration.GetSeconds();
  switch (state)
  {
    case ns3::WifiPhyState::IDLE:      gPhyIdleSec += d; break;
    case ns3::WifiPhyState::CCA_BUSY:  gPhyCcaSec  += d; break;
    case ns3::WifiPhyState::TX:        gPhyTxSec   += d; break;
    case ns3::WifiPhyState::RX:        gPhyRxSec   += d; break;
    default:                           gPhyOtherSec+= d; break;
  }
}

void CountTxBytes(Ptr<const Packet> packet)
{
  txBytes += packet->GetSize();
}

void CountRxBytes(Ptr<const Packet> packet, const Address &address)
{
  rxBytes += packet->GetSize();
}

void PsduSuccessCallback(Mac48Address address) {
  IncrementCounter(psduSucceeded, address);
}


void PsduFailCallback(Mac48Address address) {
  IncrementCounter(psduFailed, address);
}


void PhyHeaderFailCallback(Mac48Address address) {
  IncrementCounter(phyHeaderFailed, address);
}


void RxEventCallback(Mac48Address address) {
  Time now = Simulator::Now();
  if (timeFirstReceived.find(address) == timeFirstReceived.end()) {
    timeFirstReceived[address] = now;
  }
  timeLastReceived[address] = now;
}


static void PhyRxDropCallback(std::string context,
                              Ptr<const ns3::Packet> p,
                              ns3::WifiPhyRxfailureReason reason)
{
  using namespace ns3;
  rxDropReasonCounts[static_cast<int>(reason)]++;

  if (reason == WifiPhyRxfailureReason::RECEPTION_ABORTED_BY_TX) {
    abortedByTxGlobal++;
  } else if (reason == WifiPhyRxfailureReason::BUSY_DECODING_PREAMBLE
          || reason == WifiPhyRxfailureReason::PREAMBLE_DETECT_FAILURE) {
    preambleGlobal++;
  }

  WifiMacHeader hdr;
  Mac48Address addr;
  if (p) {
    Ptr<Packet> copy = p->Copy();
    if (copy->PeekHeader(hdr)) {
      addr = hdr.GetAddr2();
        switch (reason) {
          case ns3::WifiPhyRxfailureReason::L_SIG_FAILURE:
          case ns3::WifiPhyRxfailureReason::HT_SIG_FAILURE:
          case ns3::WifiPhyRxfailureReason::SIG_A_FAILURE:
          case ns3::WifiPhyRxfailureReason::SIG_B_FAILURE:
            IncrementCounter(phyHeaderFailed, addr);
            break;

          case ns3::WifiPhyRxfailureReason::BUSY_DECODING_PREAMBLE:
          case ns3::WifiPhyRxfailureReason::PREAMBLE_DETECT_FAILURE:
            IncrementCounter(rxEventWhileDecodingPreamble, addr);
            break;

          case ns3::WifiPhyRxfailureReason::RECEPTION_ABORTED_BY_TX:
            IncrementCounter(rxEventAbortedByTx, addr);
            break;

          default:
            IncrementCounter(psduFailed, addr);
            break;
        }


    }
  }
}


void PhyRxEndCallback(std::string context, Ptr<const Packet> p) {
  WifiMacHeader hdr;
  Ptr<Packet> copy = p->Copy();

  if (copy->PeekHeader(hdr)) {
    Mac48Address addr = hdr.GetAddr2();
    IncrementCounter(psduSucceeded, addr);

    if (hdr.IsBlockAck())    { IncrementCounter(blockAckRx,    addr); }
    if (hdr.IsBlockAckReq()) { IncrementCounter(blockAckReqRx, addr); }

    Time now = Simulator::Now();
    if (timeFirstReceived.find(addr) == timeFirstReceived.end()) {
      timeFirstReceived[addr] = now;
    }
    timeLastReceived[addr] = now;
  }
}



void RxEventWhileDecodingPreambleCallback(Mac48Address address) {
  IncrementCounter(rxEventWhileDecodingPreamble, address);
}

void RxEventAbortedByTxCallback(Mac48Address address) {
  IncrementCounter(rxEventAbortedByTx, address);
}


void RxWhileDecodingCallback(std::string context, Ptr<const Packet> packet)
{
    rxWhileDecodingCount++;
}


int main(int argc, char *argv[])
{

  uint32_t nWifi = 1;
  double simulationTime = 2.0;
  int mcs = 11;
  int channelWidth = 20;
  int gi = 800;
  bool pcap = false;

  CommandLine cmd;
  cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue("mcs", "use a specific MCS (0-11)", mcs);
  cmd.AddValue("nWifi", "number of stations", nWifi);
  cmd.AddValue("pcap", "Enable PCAP tracing", pcap);
  cmd.Parse(argc, argv);

  std::cout << std::endl
            << "Simulating an IEEE 802.11ax network with the following settings:" << std::endl;
  std::cout << "- number of transmitting stations: " << nWifi << std::endl;
  std::cout << "- frequency band: 5 GHz" << std::endl;
  std::cout << "- modulation and coding scheme (MCS): " << mcs << std::endl;
  std::cout << "- channel width: " << channelWidth << " MHz" << std::endl;
  std::cout << "- guard interval: " << gi << " ns" << std::endl;

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(nWifi);
  NodeContainer wifiApNode;
  wifiApNode.Create(1);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  channel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(-55.0));

  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  WifiMacHelper mac;
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211ax);
  
  std::string channelStr("{0, " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}");
  phy.Set("ChannelSettings", StringValue(channelStr));

  std::ostringstream oss;
  oss << "HeMcs" << mcs;
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(oss.str()),
                               "ControlMode", StringValue(oss.str()));

  Config::SetDefault("ns3::WifiMac::BE_MaxAmpduSize", UintegerValue(6500631));
  Config::SetDefault("ns3::WifiMac::BK_MaxAmpduSize", UintegerValue(6500631));
  Config::SetDefault("ns3::WifiMac::VI_MaxAmpduSize", UintegerValue(6500631));
  Config::SetDefault("ns3::WifiMac::VO_MaxAmpduSize", UintegerValue(6500631));

  Config::SetDefault("ns3::WifiMac::BE_BlockAckThreshold", UintegerValue(2));

  Ssid ssid = Ssid("ns3-80211ax");

  mac.SetType("ns3::StaWifiMac",
            "Ssid", SsidValue(ssid),
            "QosSupported", BooleanValue(true));


  phy.Set("RxNoiseFigure", DoubleValue(7.0));

  NetDeviceContainer staDevice;
  staDevice = wifi.Install(phy, mac, wifiStaNodes);

  for (uint32_t i = 0; i < staDevice.GetN(); ++i) {
  Ptr<ns3::WifiNetDevice> wnd = ns3::DynamicCast<ns3::WifiNetDevice>(staDevice.Get(i));
  Ptr<ns3::WifiPhy>       wph = wnd->GetPhy();
  if (i == 0) {
    wph->SetAttribute("TxPowerStart", ns3::DoubleValue(20.0));
    wph->SetAttribute("TxPowerEnd",   ns3::DoubleValue(20.0));
  } else {
    wph->SetAttribute("TxPowerStart", ns3::DoubleValue(12.0));
    wph->SetAttribute("TxPowerEnd",   ns3::DoubleValue(12.0));
  }
}

  for (uint32_t i = 0; i < wifiStaNodes.GetN(); ++i)
  {
    uint32_t nodeId = wifiStaNodes.Get(i)->GetId();

    std::ostringstream p1, p2;
    p1 << "/NodeList/" << nodeId << "/DeviceList/0/$ns3::WifiNetDevice/Phy/TxPowerStart";
    p2 << "/NodeList/" << nodeId << "/DeviceList/0/$ns3::WifiNetDevice/Phy/TxPowerEnd";

    double txDbm = (i == 0 ? 16.0 : 8.0);
    Config::Set(p1.str(), DoubleValue(txDbm));
    Config::Set(p2.str(), DoubleValue(txDbm));
  }

  mac.SetType("ns3::ApWifiMac",
            "Ssid", SsidValue(ssid),
            "QosSupported", BooleanValue(true));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install(phy, mac, wifiApNode);

  NetDeviceContainer devices;
  devices.Add(staDevice);
  devices.Add(apDevice);
    if (pcap) {
  phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.EnablePcap("pcaps/ex1-bianchi-psdu", devices, true);
  std::cout << "PCAP enabled: writing to ./pcaps/..." << std::endl;

  
}

Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                MakeCallback(&MacTxCb));
Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
                MakeCallback(&MacTxDropCb));


Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", 
                MakeCallback(&PhyRxDropCallback));


Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd", 
                MakeCallback(&PhyRxEndCallback));


Config::ConnectWithoutContext(
  "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State",
  ns3::MakeCallback(&PhyStateLogger)
);

Config::Connect(
  "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State",
  ns3::MakeCallback(&PhyStateLoggerCtx)
);
  

Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue(NanoSeconds(gi)));

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(wifiApNode);
  mobility.Install(wifiStaNodes);

  InternetStackHelper stack;
  stack.Install(wifiApNode);
  stack.Install(wifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterface;
  Ipv4InterfaceContainer apNodeInterface;

  staNodeInterface = address.Assign(staDevice);
    for (uint32_t i = 0; i < staNodeInterface.GetN(); ++i) {
    ipToNodeId[staNodeInterface.GetAddress(i)] = wifiStaNodes.Get(i)->GetId();
    }

  apNodeInterface = address.Assign(apDevice);

  ApplicationContainer sourceApplications, sinkApplications;


  InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), port);
  PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", local);

  ApplicationContainer sinkApp = sinkHelper.Install(wifiApNode.Get(0));
  sinkApp.Start(Seconds(0.0));
  sinkApp.Stop(Seconds(simulationTime + 1));

  sinkApplications.Add(sinkApp);

  Ptr<PacketSink> pktSink = DynamicCast<PacketSink>(sinkApp.Get(0));
  pktSink->TraceConnectWithoutContext("Rx", MakeCallback(&CountRx));
  //pktSink->TraceConnectWithoutContext("Rx", MakeCallback(&CountRxBytes));

  //Ptr<Application> appSink = sinkApp.Get(0);
  //appSink->TraceConnectWithoutContext("Rx", MakeCallback(&CountRx));
  //appSink->TraceConnectWithoutContext("Rx", MakeCallback(&CountRxBytes));


  for (uint32_t i = 0; i < nWifi; ++i) {
    InetSocketAddress dst(apNodeInterface.GetAddress(0), port);

    UdpClientHelper udpClient(dst);
    udpClient.SetAttribute("MaxPackets", UintegerValue(1000));
    udpClient.SetAttribute("Interval",  TimeValue(MilliSeconds(1)));
    udpClient.SetAttribute("PacketSize", UintegerValue(1472));

    ApplicationContainer srcApp = udpClient.Install(wifiStaNodes.Get(i));
    Ptr<Application> appSta = srcApp.Get(0);
    appSta->TraceConnectWithoutContext("Tx", MakeCallback(&CountTx));
    appSta->TraceConnectWithoutContext("Tx", MakeCallback(&CountTxBytes));

    appSta->SetStartTime(Seconds(1.0 + 0.02 * i));
    appSta->SetStopTime(Seconds(simulationTime + 1));
  }

  Simulator::Stop(Seconds(simulationTime + 1));

  std::clog << std::endl
            << "Starting simulation... ";

  auto start = std::chrono::high_resolution_clock::now();

  WifiTxStatsHelper txStatsHelper;
  txStatsHelper.Enable(wifiStaNodes);
  txStatsHelper.Enable(wifiApNode);
  txStatsHelper.Start(Seconds(1.0));
  txStatsHelper.Stop(Seconds(simulationTime + 1));

  WifiCoTraceHelper coHelper(Seconds(1.0), Seconds(11.0));
  coHelper.Enable(devices);

  std::vector<double> delays;
  
  Simulator::Run();
  
  coHelper.PrintStatistics(std::cout);

  std::cout << "\n*** WifiTxStatsHelper Statistics ***" << std::endl;
  std::cout << "MPDU Successes: " << txStatsHelper.GetSuccesses() << std::endl;
  std::cout << "MPDU Failures: " << txStatsHelper.GetFailures() << std::endl;
  std::cout << "MPDU Retransmissions: " << txStatsHelper.GetRetransmissions() << std::endl;
  std::cout << "*** Custom Packet Counters ***\n";
  std::cout << "- Packets sent (Tx):     " << totalPacketsSent << std::endl;
  std::cout << "- Packets received (Rx): " << totalPacketsReceived << std::endl;
  std::cout << "*** Byte Counters ***" << std::endl;
  std::cout << "- Bytes sent (Tx): " << txBytes << std::endl;
  std::cout << "- Bytes received (Rx): " << rxBytes << std::endl;
  std::cout << "\n*** Packet Loss Stats ***" << std::endl;

  int64_t sent = static_cast<int64_t>(totalPacketsSent);
  int64_t recv = static_cast<int64_t>(totalPacketsReceived);
  int64_t lost = sent - recv;
  if (lost < 0)
    lost = 0;

  double plr = (sent == 0) ? 0.0 : (100.0 * static_cast<double>(lost) / static_cast<double>(sent));

  std::cout << "- Packets sent:      " << sent << std::endl;
  std::cout << "- Packets received:  " << recv << std::endl;
  std::cout << "- Packets lost:      " << lost << std::endl;
  std::cout << "- Packet loss ratio: " << plr << " %" << std::endl;


  for (const auto& entry : rxEventWhileDecodingPreamble) {
  std::cout << "Node " << entry.first << ": RX Events While Decoding Preamble = " << entry.second << std::endl;
  }

  for (const auto& entry : rxEventAbortedByTx) {
    std::cout << "Node " << entry.first << ": RX Events Aborted By TX = " << entry.second << std::endl;
  }


  for (const auto& entry : timeFirstReceived)
  {
    std::cout << "Node " << entry.first << ": Time First RX = " << entry.second.GetSeconds() << " s" << std::endl;
  }

  for (const auto& entry : timeLastReceived) {
  std::cout << "Node " << entry.first << ": Time Last RX = " << entry.second.GetSeconds() << " s" << std::endl;
  }


  std::cout << "\n*** PSDU Failures ***" << std::endl;
  for (const auto& entry : psduFailed) {
    std::cout << "Node " << entry.first << ": PSDU Failures = " << entry.second << std::endl;
  }

  std::cout << "\n*** PHY Header Failures ***" << std::endl;
  for (const auto& entry : phyHeaderFailed) {
    std::cout << "Node " << entry.first << ": PHY Header Failures = " << entry.second << std::endl;
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::clog << ("done!") << std::endl;
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Elapsed time: " << elapsed.count() << " s\n\n";

  double throughput = 0;
  for (uint32_t index = 0; index < sinkApplications.GetN(); ++index)
  {
    uint64_t totalBytesThrough = DynamicCast<PacketSink>(sinkApplications.Get(index))->GetTotalRx();
    throughput += ((totalBytesThrough * 8) / (simulationTime * 1000000.0));
  }

  std::cout << "Results: " << std::endl;
  std::cout << "- aggregate throughput: " << throughput << " Mbit/s" << std::endl;

  if (totalPacketsReceived > 0)
  {
    Time avg = totalDelay / totalPacketsReceived;
    std::cout << "- average delay: " << std::fixed << std::setprecision(1)
              << avg.GetMicroSeconds() << " µs" << std::endl;
  }


  if (iatCount >= 1) {
    double iatVar = (iatCount > 1) ? (iatM2 / static_cast<double>(iatCount - 1)) : 0.0;
    double iatStdMs  = std::sqrt(iatVar) * 1000.0;
    double iatMeanMs = iatMean * 1000.0;
    std::cout << "- jitter (IAT std dev): " << iatStdMs  << " ms" << std::endl;
    std::cout << "- mean inter-arrival:   " << iatMeanMs << " ms" << std::endl;
  } else {
    std::cout << "- jitter (IAT std dev): N/A (too few packets)" << std::endl;
  }

  double first = std::numeric_limits<double>::infinity();
  double last  = 0.0;

  for (const auto& kv : timeFirstReceived) {
    first = std::min(first, kv.second.GetSeconds());
  }
  for (const auto& kv : timeLastReceived) {
    last = std::max(last, kv.second.GetSeconds());
  }

  if (first != std::numeric_limits<double>::infinity() && last >= first) {
    std::cout << "- data transfer duration: " << std::fixed << std::setprecision(6)
              << (last - first) << " s" << std::endl;
  }

  std::cout << "\n--- data transfer duration per receiver (MAC) ---\n";
  for (const auto& kv : timeFirstReceived)
  {
    const auto& mac = kv.first;
    auto itLast = timeLastReceived.find(mac);
    if (itLast != timeLastReceived.end())
    {
      double d = itLast->second.GetSeconds() - kv.second.GetSeconds();
      if (d >= 0.0)
      {
        std::cout << "Node " << mac << ": " << std::fixed << std::setprecision(6)
                  << d << " s\n";
      }
    }
  }


  std::cout << "\n--- Total RX events per receiver (PSDU-based) ---\n";
  for (const auto& kv : psduSucceeded)
  {
    const Mac48Address& mac = kv.first;
    uint64_t succ = kv.second;
    uint64_t fail = GetCount(psduFailed, mac);
    uint64_t total = succ + fail;

    std::cout << "Node " << mac << ": " << total
              << " (succ=" << succ << ", fail=" << fail << ")\n";
  }

  // (opcjonalnie) agregat:
  uint64_t totalRxEventsAll = 0;
  for (const auto& kv : psduSucceeded) totalRxEventsAll += kv.second;
  for (const auto& kv : psduFailed)    totalRxEventsAll += kv.second;
  std::cout << "- Total RX events (PSDU-based, aggregate): "
            << totalRxEventsAll << std::endl;


  // --- Event totals (aggregate) ---
  uint64_t psduSuccAll = 0, psduFailAll = 0;
  for (const auto& kv : psduSucceeded) psduSuccAll += kv.second;
  for (const auto& kv : psduFailed)    psduFailAll += kv.second;

  uint64_t preambleAll = 0, abortedAll = 0;
  for (const auto& kv : rxEventWhileDecodingPreamble) preambleAll += kv.second;
  for (const auto& kv : rxEventAbortedByTx)           abortedAll  += kv.second;
  preambleAll += preambleGlobal;
  abortedAll  += abortedByTxGlobal;



  uint64_t totalEvents = psduSuccAll + psduFailAll + preambleAll + abortedAll;

  std::cout << "\n--- Event totals ---\n";
  std::cout << "PSDU successes: " << psduSuccAll << "\n";
  std::cout << "PSDU failures:  " << psduFailAll << "\n";
  std::cout << "While preamble: " << preambleAll << "\n";
  std::cout << "Aborted by TX:  " << abortedAll  << "\n";
  std::cout << "Total events:   " << totalEvents << std::endl;
  std::cout << "- RX tagged:   " << rxTagged   << std::endl;
  std::cout << "- RX untagged: " << rxUntagged << std::endl;

  {
  const double total = gPhyIdleSec + gPhyCcaSec + gPhyTxSec + gPhyRxSec + gPhyOtherSec;
  const double util  = (total > 0.0) ? ((gPhyTxSec + gPhyRxSec + gPhyCcaSec) / total) : 0.0;

  std::cout << "- channel utilization: " << std::fixed << std::setprecision(2)
            << (util * 100.0) << " %" << std::endl;
  }

  std::cout << "\n--- Tx Opportunity Duration per node/device ---\n";
  for (const auto& kv : txTimePerDev)
  {
    auto key = kv.first;
    double txSec = kv.second;
    std::cout << "Node " << key.first << " Dev " << key.second
              << " | tx_op_duration=" << std::fixed << std::setprecision(6)
              << txSec << " s\n";
  }

  std::cout << "\n--- Channel Utilization per node/device ---\n";
  double obsSec = simulationTime;
  for (const auto& kv : txTimePerDev)
  {
    auto key = kv.first;
    double tx = kv.second;
    double rx = 0.0, cca = 0.0;

    auto itRx  = rxTimePerDev.find(key);
    if (itRx != rxTimePerDev.end())  rx = itRx->second;

    auto itCca = ccaTimePerDev.find(key);
    if (itCca != ccaTimePerDev.end()) cca = itCca->second;

    double busy = tx + rx + cca;
    double utilPct = (obsSec > 0.0) ? (busy / obsSec * 100.0) : 0.0;

    std::cout << "Node " << key.first << " Dev " << key.second
              << " | busy=" << std::fixed << std::setprecision(6) << busy << " s"
              << " (" << std::setprecision(2) << utilPct << " %)\n";
  }


  std::cout << "\n--- Block ACK metrics ---\n";
  for (const auto& kv : blockAckRx) {
    std::cout << "Node " << kv.first << " : BA received = " << kv.second << "\n";
  }
  for (const auto& kv : blockAckReqRx) {
    std::cout << "Node " << kv.first << " : BAR received = " << kv.second << "\n";
  }
  uint64_t baAll = 0, barAll = 0;
  for (const auto& kv : blockAckRx)    baAll  += kv.second;
  for (const auto& kv : blockAckReqRx) barAll += kv.second;
  std::cout << "- BA total:  " << baAll  << "\n";
  std::cout << "- BAR total: " << barAll << std::endl;

  std::cout << "\n--- Frame Aggregation Metrics (proxy) ---\n";

  uint64_t psduSuccAllProxy = 0;
  for (const auto& kv : psduSucceeded) psduSuccAllProxy += kv.second;

  uint64_t baAllProxy = 0, barAllProxy = 0;
  for (const auto& kv : blockAckRx)    baAllProxy  += kv.second;
  for (const auto& kv : blockAckReqRx) barAllProxy += kv.second;

  double aggSharePct = (psduSuccAllProxy > 0)
    ? (static_cast<double>(baAllProxy) / static_cast<double>(psduSuccAllProxy) * 100.0)
    : 0.0;


  std::cout << "- PSDU successes: " << psduSuccAllProxy << "\n";
  std::cout << "- BA total:       " << baAllProxy  << "  (lower bound on aggregated bursts)\n";
  std::cout << "- BAR total:      " << barAllProxy << "\n";
  std::cout << "- Aggregated bursts share (proxy): "
            << std::fixed << std::setprecision(2) << aggSharePct << " %\n";

  std::cout << "\n--- PHY RX drop reasons (debug) ---\n";
  for (const auto& kv : rxDropReasonCounts)
  {
    std::cout << "reason=" << kv.first << " | count=" << kv.second << "\n";
  }



  std::cout << "\n--- Throughput per node ---" << std::endl;
  for (auto const &entry : bytesReceivedPerNode)
  {
      double throughputMbps = (entry.second * 8.0) / (simulationTime * 1e6);
      std::cout << "Node " << entry.first << ": " << throughputMbps << " Mbit/s" << std::endl;
  }

  std::cout << "\n--- Attempts / Retries / Failures per node/device ---\n";
  uint64_t totAtt = 0, totRet = 0, totDrop = 0;
  for (const auto& kv : macTxAttempts)
  {
    auto key = kv.first;
    uint64_t att = kv.second;
    uint64_t ret = macTxRetries[key];
    uint64_t drp = macTxDrops[key];

    std::cout << "Node " << key.first << " Dev " << key.second
              << " | attempts=" << att
              << ", retries="   << ret
              << ", failures="  << drp << "\n";

    totAtt += att; totRet += ret; totDrop += drp;
  }


  std::cout << "\n--- Tx Attempts / Retries / Failures (aggregate) ---\n";
  std::cout << "Total Attempts: " << totAtt << "\n";
  std::cout << "Total Retries:  " << totRet << "\n";
  std::cout << "Total Failures: " << totDrop << "\n";

  std::cout << "\n*** PSDU Success (per receiver) ***\n";
  for (const auto& kv : psduSucceeded) {
    std::cout << "Node " << kv.first << ": PSDU Success = " << kv.second << "\n";
  }


  Simulator::Destroy();

  
  return 0;
  
}
