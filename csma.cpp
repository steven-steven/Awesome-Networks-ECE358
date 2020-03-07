#include <iostream>
#include <random>
#include <queue>
#include <deque>
#include <vector>
#include <algorithm>
#include <tuple>
#include <fstream>
#include <cmath>

using namespace std;

//initializing
std::random_device rd;
uniform_real_distribution<double> distribution(0.0,1.0);
default_random_engine generator(rd());

// ORIGINAL PARAMETERS
double FRAME_LEN = 1500;
double PROP_DELAY = 0.00000005;
double TRANSMISSION_SPEED = 1000000;
double BACKOFF = 512/TRANSMISSION_SPEED; ///constant unit of wait-time = t to transfer 512 bits (used for exp backoff)
bool PERSISTENT_SENSING = false;

/*
  Generate eponential distribution
*/
double generateRandom(double mean){
  double uniformRandomVar = distribution(generator);
  //expRandomVar 
  return -(1/mean)*log(1-uniformRandomVar);
}

struct Node {
  deque<double> queue;
  int collisionCounter = 0;
  int sensingCounter = 0;  //count number of times this node failed to sense medium (non-persistent CSMA/CD)
};

/*
/  While the packet arrives before transmission is complete, the node will continue to sense the medium and backoff.
/  Following code simulate this by implementing the backoff multiple times for a particular node until the arrival-time is when medium is free
/  the exponential backoff will also apply to the sender node
*/
void nonPersistentSensing(Node &node, const double timeAfterTransmission){
  //while the node still needs to wait till the end of transmission.. backoff
  while(node.queue.front() < timeAfterTransmission){
    
    // Increment and Drop leading packet if > 10
    node.sensingCounter++;
    if(node.sensingCounter >= 10){
      node.queue.pop_front();
      node.sensingCounter = 0;
    }
    
    // Enter exponential backoff and calculate new departure time
    int randomNumber = distribution(generator)* (pow(2, node.sensingCounter)-1);  //uniforDistr over 0 to 2^i-1
    double Twaiting = randomNumber*BACKOFF;
    node.queue.at(0) += Twaiting;

  }
}

// OBSERVATION: there is no Interframe Gap
// If traffic is busy, one node might continually send, starving the rest; this is probably intended
void persistentSensing(Node &node, double timeAfterTransmission){
  if (node.queue.front() < timeAfterTransmission){
    //update packet arrival to the time after sender finish
    node.queue.front() = timeAfterTransmission;
  }
}

/*
/  Initialize a shared bus array containing some number of Node objects.
/  Generate arrival times for each node in the queue
*/
Node* normalSetup(const int& nodeCount, const double& Tsim, const double& ratePktPerSec) {
	Node* bus = new Node[nodeCount];
	for (int i = 0; i < nodeCount; i++) {
		Node* newNode = new Node();
		double arrivalTime = 0;
		while (arrivalTime < Tsim + 2) {
			arrivalTime += generateRandom(ratePktPerSec);
			newNode->queue.push_back(arrivalTime);
		}
		bus[i] = *newNode;
	}
	return bus;
}

// Run Simulation
void csmaSimulation(const int nodeCount, double Tsim, double transmissionDelay, Node* bus, double& efficiency, double& throughput){
	
  int countTransmitted = 0;
  int countSuccess = 0;
  
  //run simulation (until Tsim)
  double senderTime;
  while(true){

    //for all nodes, select next sender (the earliest arrival time form all the nodes' queue)
    int senderNode = 0;
    for(int i = 1; i<nodeCount; i++){
      if(bus[i].queue.front() < bus[senderNode].queue.front()){
        senderNode = i;
      }
    }
    senderTime = bus[senderNode].queue.front();

    if(senderTime>=Tsim) break;  //end simulation if earliest arrivalTime goes over Tsim
    
    //check all non-sender nodes if there will be conflict
    vector<int> conflictingNodes; 
    for(int i = 0; i<nodeCount; i++){
      if(i == senderNode) continue;  
      if(bus[i].queue.front() <= (senderTime + PROP_DELAY*abs(nodeCount-senderNode))){
        //this node will cause conflict
        conflictingNodes.push_back(i);
      }
    }
    
    if(conflictingNodes.size() > 0){  // A COLLISION
      // Update transmitted-attempt count - include all nodes that transmitted a packet as well as the sending node itself
	    countTransmitted += (conflictingNodes.size() + 1);
      
      // iterate collided nodes. Update their counters and update backoff time
      int maxNodeOffset = 0;
      for(int i = 0; i<conflictingNodes.size(); i++){
        int conflictIndex = conflictingNodes[i];
        maxNodeOffset = max(maxNodeOffset, abs(conflictIndex-senderNode));  //furthest node to sender distance

        //for non-persistent sensing, reset sensing count
        if( !PERSISTENT_SENSING )
          bus[conflictIndex].sensingCounter = 0;
        
        //for each conflicting node, inc and drop if collision count = 10
        if(++bus[conflictIndex].collisionCounter >= 10){
          bus[conflictIndex].queue.pop_front();  //drop
          bus[conflictIndex].collisionCounter = 0;
        }
        
        //calculate random wait time
        int randomNumber = distribution(generator)* (pow( 2, bus[conflictIndex].collisionCounter )-1);  //uniforDistr over 0 to 2^i-1
        double Twaiting = randomNumber*BACKOFF;
        
        // update pkt arrival times to end of random wait time
        double endOfWait = senderTime + (PROP_DELAY*abs(conflictIndex-senderNode)) + Twaiting;
        if (bus[conflictIndex].queue.front() < endOfWait) {
          bus[conflictIndex].queue.front() = endOfWait;
        }

      }

      //for non-persistent sensing, reset sensing count
      if( !PERSISTENT_SENSING )
        bus[senderNode].sensingCounter = 0;
      
      //inc and drop sender node if collision count = 10
      if(++bus[senderNode].collisionCounter >= 10){
        bus[senderNode].queue.pop_front();
        bus[senderNode].collisionCounter = 0;
      }

      // update pkt arrival times to end of random wait time
      int randomNumber = distribution(generator)* (pow( 2, bus[senderNode].collisionCounter )-1);  //uniforDistr over 0 to 2^i-1
      double Twaiting = randomNumber*BACKOFF;
      double endOfWait = senderTime + ((maxNodeOffset) * PROP_DELAY) + Twaiting;
      if (bus[senderNode].queue.front() < endOfWait) {
        bus[senderNode].queue.front() = endOfWait;
      }
      
    } else {  // NO COLLISION. SENDER SUCCEEDS
    
      //Update counter and pop sender
      countSuccess++;
      countTransmitted++;
      bus[senderNode].queue.pop_front();
      bus[senderNode].sensingCounter = 0; // (Non-PERSISTENT CSMA)
      bus[senderNode].collisionCounter = 0;
      
      //Update arrival times of all nodes depending on sensing strategy
      if(PERSISTENT_SENSING){
        // for persistent sensing, all nodes will wait till the end of transmission. Update the arrival time.
        for(int i = 0; i<nodeCount; i++){
          
          double timeAfterTransmission = senderTime + (transmissionDelay) + PROP_DELAY*abs(i-senderNode);
          // SENSING in persistent
          persistentSensing(bus[i], timeAfterTransmission);
        }
        
      } else {
        //for non-persistent sensing, execute non-persistent sensing (with backoffs) for non-sender node.
        //if it's a sender node, do persistent sensing (since we popped off the sent node, we want to move the next timestamp forward to present)
        double timeAfterTransmission;
        for(int i = 0; i<senderNode; i++){
          timeAfterTransmission = senderTime + (transmissionDelay) + PROP_DELAY*abs(i-senderNode);
          nonPersistentSensing(bus[i], timeAfterTransmission);
        }
        timeAfterTransmission = senderTime + transmissionDelay;
        persistentSensing(bus[senderNode], timeAfterTransmission);
        for(int i = senderNode+1; i<nodeCount; i++){
          timeAfterTransmission = senderTime + (transmissionDelay) + PROP_DELAY*abs(i-senderNode);
          nonPersistentSensing(bus[i], timeAfterTransmission);
        }
      }
    }
    
  }
  
  //update returned result
  efficiency = (double)countSuccess/(double)countTransmitted;
  throughput = countSuccess * FRAME_LEN / Tsim;
}

void normalRun(const int nodeCount, double Tsim, double transmissionDelay, double ratePktPerSec, double& efficiency, double& throughput) {
	Node* bus = normalSetup(nodeCount, Tsim, ratePktPerSec);  //initialize bus with nodes
	csmaSimulation(nodeCount, Tsim, transmissionDelay, bus, efficiency, throughput);  //run simulation
}


int main(){

  double Tsim = 1000;
  double transmissionDelay = FRAME_LEN/TRANSMISSION_SPEED; //L/R
  
  ofstream ThroughputData;
  ofstream EfficiencyData;
  ofstream ThroughputData_nonPer;
  ofstream EfficiencyData_nonPer;
  EfficiencyData.open("Efficiency_persistent.csv"); 
  ThroughputData.open("Throughput_persistent.csv"); 
  EfficiencyData_nonPer.open("Efficiency_nonPersistent.csv"); 
  ThroughputData_nonPer.open("Throughput_nonPersistent.csv"); 
	EfficiencyData << "Number of Nodes, Efficiency (7pkt/sec), Efficiency (10pkt/sec), Efficiency (20pkt/sec)" << endl;
  ThroughputData << "Number of Nodes, Throughput (7pkt/sec), Throughput (10pkt/sec), Throughput (20pkt/sec)" << endl;
  EfficiencyData_nonPer << "Number of Nodes, Efficiency (7pkt/sec), Efficiency (10pkt/sec), Efficiency (20pkt/sec)" << endl;
  ThroughputData_nonPer << "Number of Nodes, Throughput (7pkt/sec), Throughput (10pkt/sec), Throughput (20pkt/sec)" << endl;
  
  double efficiency7;
  double efficiency10;
  double efficiency20;
  double throughput7;
  double throughput10;
  double throughput20;
  for (int NODE_COUNT = 20; NODE_COUNT <= 100; NODE_COUNT += 20) {
    
    PERSISTENT_SENSING = true;
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 7, efficiency7, throughput7);
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 10, efficiency10, throughput10);
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 20, efficiency20, throughput20);
    
    EfficiencyData << NODE_COUNT << ",";
  	EfficiencyData << efficiency7 << ",";
    EfficiencyData << efficiency10 << ",";
    EfficiencyData << efficiency20 << endl;
    ThroughputData << NODE_COUNT << ",";
  	ThroughputData << throughput7 << ",";
    ThroughputData << throughput10 << ",";
    ThroughputData << throughput20 << endl;

    PERSISTENT_SENSING = false;
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 7, efficiency7, throughput7);
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 10, efficiency10, throughput10);
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 20, efficiency20, throughput20);
    
    EfficiencyData_nonPer << NODE_COUNT << ",";
  	EfficiencyData_nonPer << efficiency7 << ",";
    EfficiencyData_nonPer << efficiency10 << ",";
    EfficiencyData_nonPer << efficiency20 << endl;
    ThroughputData_nonPer << NODE_COUNT << ",";
  	ThroughputData_nonPer << throughput7 << ",";
    ThroughputData_nonPer << throughput10 << ",";
    ThroughputData_nonPer << throughput20 << endl;
  }
   
  EfficiencyData.close();
  ThroughputData.close();
  EfficiencyData_nonPer.close();
  ThroughputData_nonPer.close();
  return 0;
}