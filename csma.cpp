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


// TEST PARAMETERS
//double PROP_DELAY = 0;
//double TRANSMISSION_SPEED = 1000000;
//double BACKOFF = 512/TRANSMISSION_SPEED; ///constant unit of wait-time = t to transfer 512 bits (used for exp backoff)
//bool PERSISTENT_SENSING = true;

// ORIGINAL PARAMETERS
double FRAME_LEN = 1500;
double PROP_DELAY = 0.00000005;
double TRANSMISSION_SPEED = 1000000;
double BACKOFF = 512/TRANSMISSION_SPEED; ///constant unit of wait-time = t to transfer 512 bits (used for exp backoff)
bool PERSISTENT_SENSING = true;

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
  Print queue
*/
void printQueue(deque<double> q)
{
  cout<<"size " << q.size()<<endl;
	//printing content of queue 
	while (!q.empty()){
   cout<<" "<<q.front();
		q.pop_front();
	}
	cout<<endl;
 cout<<endl;
 cout<<endl;
}


// the exponential backoff will also apply to the sender node
void nonPersistentSensing(Node &node, const double timeAfterTransmission){
  while(node.queue.front() < timeAfterTransmission){
    node.sensingCounter++;
    
    // Drop leading packet
    if(node.sensingCounter >= 10){
      node.queue.pop_front();
    }
    // Enter exponential backoff and calculate new departure time
    int randomNumber = distribution(generator)* (pow(2, node.sensingCounter)-1);  //uniforDistr over 0 to 2^i-1
    double Twaiting = randomNumber*BACKOFF;
    node.queue.at(0) += Twaiting;

    if (node.sensingCounter >= 10) {
      node.sensingCounter = 0;
    }
  }
}

// OBSERVATION: there is no Interframe Gap
// If traffic is busy, one node might continually send, starving the rest; this is probably intended
void persistentSensing(Node &node, double timeAfterTransmission){
  if (node.queue.front() < timeAfterTransmission){
    //update packet arrival to the time after sender finish
    ///////////////cout<<"update to "<<timeAfterTransmission<<endl;
    node.queue.front() = timeAfterTransmission;
  }
}

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

Node* test1Setup(const int& nodeCount) {

	Node* bus = new Node[nodeCount];
	bus[0].queue.push_back(5);
	bus[0].queue.push_back(6);
	bus[0].queue.push_back(7);
	bus[0].queue.push_back(8);



	bus[0].queue.push_back(60);
	bus[0].queue.push_back(80);
	bus[0].queue.push_back(88);
	bus[0].queue.push_back(89);
	bus[0].queue.push_back(99);
	bus[0].queue.push_back(100);


	bus[1].queue.push_back(6);
	bus[1].queue.push_back(14);

	bus[1].queue.push_back(16);
	bus[1].queue.push_back(18);
	bus[1].queue.push_back(27);
	bus[1].queue.push_back(30);
	bus[1].queue.push_back(66);
	bus[1].queue.push_back(68);
	
	return bus;
}


// Run Simulation
void csmaSimulation(const int nodeCount, double Tsim, double transmissionDelay, Node* bus, double& efficiency, double& throughput){
	
  int countTransmitted = 0;
  int countSuccess = 0;
  
  //run simulation (until Tsim)
  double senderTime;
  while(true){

    int senderNode = 0;
    for(int i = 1; i<nodeCount; i++){
      //for all nodes, select sender (min arrival time)
      //////////////////cout<<"NODE "<<i<<endl;
      //////////////////printQueue(bus[i].queue);
       
      if(bus[i].queue.front() < bus[senderNode].queue.front()){
        senderNode = i;
      }
    }
    senderTime = bus[senderNode].queue.front();

    if(senderTime>=Tsim) break;  //end simulation if earliest arrivalTime goes over Tsim
    
    vector<int> conflictingNodes; 
    for(int i = 0; i<nodeCount; i++){
      //check all non-sender nodes if there'll be conflict
      if(i == senderNode) continue;
      if(bus[i].queue.front() < (senderTime + PROP_DELAY*abs(nodeCount-senderNode))){
        //this node will cause conflict (sent)
        conflictingNodes.push_back(i);
      }
    }
    
    if(conflictingNodes.size() > 0){  //there is collision
      // Include all nodes that transmitted a packet as well as the sending node itself
	    countTransmitted += (conflictingNodes.size() + 1);
      
      int maxNodeOffset = 0;
      for(int i = 0; i<conflictingNodes.size(); i++){
        int conflictIndex = conflictingNodes[i];
        maxNodeOffset = max(maxNodeOffset, abs(conflictIndex-senderNode));

        bus[conflictIndex].sensingCounter = 0;
        
        //for each conflicting node, inc and drop if count = 10
        if(++bus[conflictIndex].collisionCounter >= 10){
          //drop
          bus[conflictIndex].queue.pop_front();
        }
        
        //calculate random wait time
        int randomNumber = distribution(generator)* (pow( 2, bus[conflictIndex].collisionCounter )-1);  //uniforDistr over 0 to 2^i-1
        double Twaiting = randomNumber*BACKOFF;
        
        // update pkt arrival times to end of random wait time
        double endOfWait = senderTime + (PROP_DELAY*abs(conflictIndex-senderNode)) + Twaiting;
        // Collision is detected 
        if (bus[conflictIndex].queue.front() < endOfWait) {
          bus[conflictIndex].queue.front() = endOfWait;
        }
        
        if (bus[conflictIndex].collisionCounter >= 10) {
          bus[conflictIndex].collisionCounter = 0;
        }
      }

      bus[senderNode].sensingCounter = 0;
      
      if(++bus[senderNode].collisionCounter >= 10){
        //drop
        bus[senderNode].queue.pop_front();
      }

      int randomNumber = distribution(generator)* (pow( 2, bus[senderNode].collisionCounter )-1);  //uniforDistr over 0 to 2^i-1
      double Twaiting = randomNumber*BACKOFF;

      double endOfWait = senderTime + ((maxNodeOffset) * PROP_DELAY) + Twaiting;

      // Collision is detected 
      if (bus[senderNode].queue.front() < endOfWait) {
        bus[senderNode].queue.front() = endOfWait;
      }

      if (bus[senderNode].collisionCounter >= 10) {
        bus[senderNode].collisionCounter = 0;
      }
    } else {
      //no conflict. Sender succeeds. Reset counter of sender.
      countSuccess++;
      countTransmitted++;
      bus[senderNode].queue.pop_front();
      bus[senderNode].sensingCounter = 0; // (Non-PERSISTENT CSMA)
      bus[senderNode].collisionCounter = 0;
      
      //Update arrival times of all nodes depending on sensing strategy
      if(PERSISTENT_SENSING){
        for(int i = 0; i<nodeCount; i++){
          
          double timeAfterTransmission = senderTime + (transmissionDelay) + PROP_DELAY*abs(i-senderNode);
          // SENSING in persistent
          persistentSensing(bus[i], timeAfterTransmission);
        }
      } else {
        double timeAfterTransmission;
        for(int i = 0; i<senderNode; i++){
          timeAfterTransmission = senderTime + (transmissionDelay) + PROP_DELAY*abs(i-senderNode);
          // SENSING in non-persistent CSMA/CD
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
  
  efficiency = (double)countSuccess/(double)countTransmitted;
  throughput = countSuccess * FRAME_LEN / Tsim;
}


void test1Run(const int nodeCount, double Tsim, double transmissionDelay, double& efficiency, double& throughput) {
	Node* bus = test1Setup(nodeCount);
	return csmaSimulation(nodeCount, Tsim, transmissionDelay, bus, efficiency, throughput);
}

void normalRun(const int nodeCount, double Tsim, double transmissionDelay, double ratePktPerSec, double& efficiency, double& throughput) {
	Node* bus = normalSetup(nodeCount, Tsim, ratePktPerSec);
	csmaSimulation(nodeCount, Tsim, transmissionDelay, bus, efficiency, throughput);
}


int main(){

  double Tsim = 1000;
  double transmissionDelay = FRAME_LEN/TRANSMISSION_SPEED; //L/R
  
  ofstream ThroughputData;
  ofstream EfficiencyData;
  EfficiencyData.open("Efficiency_3.csv"); 
  ThroughputData.open("Throughput_4.csv"); 
	EfficiencyData << "Number of Nodes, Efficiency (7pkt/sec), Efficiency (10pkt/sec), Efficiency (20pkt/sec)" << endl;
  ThroughputData << "Number of Nodes, Throughput (7pkt/sec), Throughput (10pkt/sec), Throughput (20pkt/sec)" << endl;
  
  //Test Finite Buffer
  double efficiency7;
  double efficiency10;
  double efficiency20;
  double throughput7;
  double throughput10;
  double throughput20;
  for (int NODE_COUNT = 20; NODE_COUNT <= 100; NODE_COUNT += 20) {
    
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 7, efficiency7, throughput7);
    //cout<< "efficiency7 "<<efficiency7<<endl;
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 10, efficiency10, throughput10);
    //cout<< "efficiency10 "<<efficiency10<<endl;
    normalRun(NODE_COUNT, Tsim, transmissionDelay, 20, efficiency20, throughput20);
    //cout<< "efficiency20 "<<efficiency20<<endl;
    
    EfficiencyData << NODE_COUNT << ",";
  	EfficiencyData << efficiency7 << ",";
    EfficiencyData << efficiency10 << ",";
    EfficiencyData << efficiency20 << endl;

    ThroughputData << NODE_COUNT << ",";
  	ThroughputData << throughput7 << ",";
    ThroughputData << throughput10 << ",";
    ThroughputData << throughput20 << endl;
  }
   
  EfficiencyData.close();
  ThroughputData.close();

	// Test
	//const int NODE_COUNT = 2;
	//double Tsim = 400;
	//double transmissionDelay = 1500 / TRANSMISSION_SPEED; //L/R

	//test1Run(NODE_COUNT, Tsim, 7);

  return 0;
}
