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
double gerateRandom();
void checkExpDistribution();

//initializing
std::random_device rd;
uniform_real_distribution<double> distribution(0.0,1.0);
default_random_engine generator(rd());

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

// All the packets are now sandwitched together at the same time
void persistentSensing(Node &node, double timeAfterTransmission){
  //loop through this node's queue to update all arrivalTime during the transmission. Simulate POLLING
  int queuePos = 0;
  while(node.queue.at(queuePos) < timeAfterTransmission){
    //update packet arrival to the time after sender finish
    ///////////////cout<<"update to "<<timeAfterTransmission<<endl;
    node.queue.at(queuePos) = timeAfterTransmission;
    queuePos++;
  }
}

//return efficiency
double csmaSimulation(const int nodeCount, double Tsim, double transmissionDelay, double ratePktPerSec){

  int countTransmitted = 0;
  int countSuccess = 0;
  
  //initialize nodes
  Node* bus = new Node[nodeCount];
  for(int i = 0; i<nodeCount; i++){
    Node* newNode = new Node();
    double arrivalTime = 0;
    while(arrivalTime<Tsim+2){
      arrivalTime += generateRandom(ratePktPerSec);
      newNode->queue.push_back(arrivalTime);
    }
    bus[i] = *newNode;
  }
  
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
        
        //for each conflicting node, inc and drop if count > 0
        if(++bus[conflictIndex].collisionCounter > 10){
          //drop
          bus[conflictIndex].queue.pop_front();
          bus[conflictIndex].collisionCounter = 0;
        
          continue;
        }
        
        //calculate random wait time
        int randomNumber = distribution(generator)* (pow( 2, bus[conflictIndex].collisionCounter )-1);  //uniforDistr over 0 to 2^i-1
        double Twaiting = randomNumber*BACKOFF;
        
        //update pkt arrival times to end of random wait time
        int queuePos = 0;
        // Assumption is that senderTime is time at which collision is detected by all nodes
        double endOfWait = senderTime + Twaiting;
        // TODO: will CSMA sensing happen properly ?
        // Collision is detected 
        if (bus[conflictIndex].queue.at(queuePos) < endOfWait) {
          bus[conflictIndex].queue.at(queuePos) = endOfWait;
        }
        for (queuePos = 1; 
            queuePos < bus[conflictIndex].queue.size()
            && bus[conflictIndex].queue.at(queuePos) < endOfWait; 
            queuePos++
          ) 
        {
            bus[conflictIndex].queue.at(queuePos) = endOfWait;
        }
      }
    } else {
      // TODO: Test
      // For current node, delay the transmission of any packet that is too close to the one being sent 
      for ( int i = 1; 
            (i < bus[senderNode].queue.size()) 
            && (bus[senderNode].queue.at(i) < (transmissionDelay*i + senderTime) ); 
            i++
          ) 
      {
        bus[senderNode].queue.at(i) = transmissionDelay*i + senderTime;
      }
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
  
  //cout<< "countTransmitted "<<countTransmitted<<endl;
  //cout<< "countSuccess "<<countSuccess<<endl;
  
  return (double)countSuccess/(double)countTransmitted;
}


int main(){

  //int nodeCount = 20;
  double Tsim = 1000;
  double transmissionDelay = 1500/TRANSMISSION_SPEED; //L/R
  //double ratePktPerSec = 12;
  
  //double efficiency = csmaSimulation(nodeCount, Tsim, transmissionDelay, ratePktPerSec);
  // cout<< "efficiency7 "<<efficiency<<endl;
  
  ofstream myfile;
	ofstream errorCount;
  myfile.open("five.csv"); 
	myfile << "Number of Nodes, Efficiency (7pkt/sec), Efficiency (10pkt/sec)" << endl;
  
  
  //Test Finite Buffer
  for (int NODE_COUNT = 20; NODE_COUNT <= 100; NODE_COUNT += 20) {
    
    double efficiency7 = csmaSimulation(NODE_COUNT, Tsim, transmissionDelay, 7);
    cout<< "efficiency7 "<<efficiency7<<endl;
    double efficiency10 = csmaSimulation(NODE_COUNT, Tsim, transmissionDelay, 10);
    cout<< "efficiency10 "<<efficiency10<<endl;
    double efficiency20 = csmaSimulation(NODE_COUNT, Tsim, transmissionDelay, 20);
    cout<< "efficiency20 "<<efficiency20<<endl;
    
    myfile << NODE_COUNT << ",";
  	myfile << efficiency7 << ",";
    myfile << efficiency10 << endl;
    myfile << efficiency20 << endl;
  }
   
  myfile.close();
  
  return 0;
}
