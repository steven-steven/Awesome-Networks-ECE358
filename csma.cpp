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

void nonPersistentSensing(Node &node, double timeAfterTransmission, int &countTransmitted){
  while(node.queue.front() < timeAfterTransmission){
    node.sensingCounter++;
    
    if(node.sensingCounter >= 10){
      //drop
      cout<<"DROOOPP"<<endl;
      node.queue.pop_front();
      node.sensingCounter = 0;
      countTransmitted++;
      continue;
    }
    //update if not drop
    int randomNumber = distribution(generator)* (pow(2, node.sensingCounter)-1);  //uniforDistr over 0 to 2^i-1
    double Twaiting = randomNumber*BACKOFF;
    node.queue.at(0) += Twaiting;
  }
  
  //update other N-1 packets in queue that's < front
  int queuePos = 1;
  while(node.queue.at(queuePos) < node.queue.front()){
    node.queue.at(queuePos) = node.queue.front();
    queuePos++;
  }
}

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


//return efficiency
double csmaSimulation(const int nodeCount, double Tsim, double transmissionDelay, Node* bus){
	
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
      
      // TODO: 
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
      //include the sender as a conflicting node
      conflictingNodes.push_back(senderNode);
	  countTransmitted += conflictingNodes.size();
      
      for(int i = 0; i<conflictingNodes.size(); i++){
        int conflictIndex = conflictingNodes[i];
        bus[conflictIndex].sensingCounter = 0; // (Non-PERSISTENT CSMA)
        
        //for each conflicting node, inc and drop if count = 10
        if(++bus[conflictIndex].collisionCounter >= 10){
          //drop
          bus[conflictIndex].queue.pop_front();
        }
        
        //calculate random wait time
        int randomNumber = distribution(generator)* (pow( 2, bus[conflictIndex].collisionCounter )-1);  //uniforDistr over 0 to 2^i-1
        double Twaiting = randomNumber*BACKOFF;
        
        //update pkt arrival times to end of random wait time
        // Assumption is that senderTime is time at which collision is detected by all nodes
        double endOfWait = senderTime + Twaiting;
        // Collision is detected 
        if (bus[conflictIndex].queue.front() < endOfWait) {
          bus[conflictIndex].queue.front() = endOfWait;
        }
		if (bus[conflictIndex].collisionCounter >= 10) {
			bus[conflictIndex].collisionCounter = 0;
		}
      }
    } else {
      //no conflict. Sender succeeds. Reset counter of sender.
      countSuccess++;
      countTransmitted++;
      bus[senderNode].queue.pop_front();
      bus[senderNode].sensingCounter = 0; // (Non-PERSISTENT CSMA)
      bus[senderNode].collisionCounter = 0;
      
      //Update arrival times of all nodes depending on sensing strategy
      for(int i = 0; i<nodeCount; i++){
        
        double timeAfterTransmission = senderTime + (transmissionDelay) + PROP_DELAY*abs(nodeCount-senderNode);
        
        if(PERSISTENT_SENSING){
          // SENSING in persistent
          persistentSensing(bus[i], timeAfterTransmission);
        }else{
          // SENSING in non-persistent CSMA/CD
          nonPersistentSensing(bus[i], timeAfterTransmission, countTransmitted);
        }
      }
    }
    
  }
  
  return (double)countSuccess/(double)countTransmitted;
}


double test1Run(const int nodeCount, double Tsim, double transmissionDelay) {
	Node* bus = test1Setup(nodeCount);
	return csmaSimulation(nodeCount, Tsim, transmissionDelay, bus);
}

double normalRun(const int nodeCount, double Tsim, double transmissionDelay, double ratePktPerSec) {
	Node* bus = normalSetup(nodeCount, Tsim, ratePktPerSec);
	return csmaSimulation(nodeCount, Tsim, transmissionDelay, bus);
}


int main(){

  double Tsim = 1000;
  double transmissionDelay = 1500/TRANSMISSION_SPEED; //L/R
  
  ofstream myfile;
	ofstream errorCount;
  myfile.open("two.csv"); 
	myfile << "Number of Nodes, Efficiency (7pkt/sec), Efficiency (10pkt/sec), Efficiency (20pkt/sec)" << endl;
  
  
  //Test Finite Buffer
  for (int NODE_COUNT = 20; NODE_COUNT <= 100; NODE_COUNT += 20) {
    
    double efficiency7 = normalRun(NODE_COUNT, Tsim, transmissionDelay, 7);
    //cout<< "efficiency7 "<<efficiency7<<endl;
    double efficiency10 = normalRun(NODE_COUNT, Tsim, transmissionDelay, 10);
    //cout<< "efficiency10 "<<efficiency10<<endl;
    double efficiency20 = normalRun(NODE_COUNT, Tsim, transmissionDelay, 20);
    //cout<< "efficiency20 "<<efficiency20<<endl;
    
    myfile << NODE_COUNT << ",";
  	myfile << efficiency7 << ",";
    myfile << efficiency10 << ",";
    myfile << efficiency20 << endl;
  }
   
  myfile.close();

	// Test
	//const int NODE_COUNT = 2;
	//double Tsim = 400;
	//double transmissionDelay = 1500 / TRANSMISSION_SPEED; //L/R

	//test1Run(NODE_COUNT, Tsim, 7);

  return 0;
}
