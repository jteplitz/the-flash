#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

#define MTU 1500
#define ESCAPE_LAMBDA 1
#define SIGMA 200
// Observe and update distributions each tick (20 ms)
#define TICK 20
// TICK expressed in seconds
#define TAU 0.02


using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_(debug), the_window_size(1.0), num_packets_received(0), num_packets_sent(0),
    num_packets_tick_start(0), num_packets_tick_mutex(), startedThread(false)
{
  debug_ = false;
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(0,1000);
  std::set<unsigned int> usedLabels;
  for (int i = 0; i < 256; i++) {
    int label = distribution(generator);
    while (usedLabels.find(label) != usedLabels.end()) {
      label = distribution(generator);
    }
    usedLabels.insert(label);
    lambdas[i].label = label;
    lambdas[i].prob = 1.0/256;
    lambdas[i].score = 1.0/256;
  }
  for (int i = 0; i < 256; i++) {
    cout << lambdas[i].label << ", ";
  }
  cout << endl;
}

unsigned int Controller::queue_occupancy_est( void ) {
  return num_packets_sent - num_packets_received;
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */
  

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return the_window_size > 1 ? (int)the_window_size : 1;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
  num_packets_sent++;
  if (!startedThread) {
    startedThread = true;
    std::thread updateThread(&Controller::update_tick, this);
    updateThread.detach();
  }
}

int factorial(int someNumber) {
  int curr = 1;
  while (someNumber > 0) {
    curr *= someNumber;
    someNumber--;
  }
  return curr;
}

// static int sortByProb_asc(const void *one, const void *two) {
//   lambdaEntry *lambdaOne = (lambdaEntry*)one;
//   lambdaEntry *lambdaTwo = (lambdaEntry*)two;
  
//   if (lambdaTwo->prob == lambdaOne->prob){
//     return 0;
//   }
//   else if (lambdaOne->prob < lambdaTwo->prob) {
//     return -1;
//   } 
//   else {
//     return 1;
//   }
// }

static int sortByProb_desc(const void *one, const void *two) {
  lambdaEntry *lambdaOne = (lambdaEntry*)one;
  lambdaEntry *lambdaTwo = (lambdaEntry*)two;
  if (lambdaTwo->prob == lambdaOne->prob){
    return 0;
  }
  else if (lambdaTwo->prob > lambdaOne->prob) {
    return 1;
  } 
  else {
    return -1;
  }
}

void Controller::update_tick() {
  while (true) {
    num_packets_tick_mutex.lock();
    int numberOfPacketsInTick = num_packets_tick_start;
    num_packets_tick_start = 0;
    num_packets_tick_mutex.unlock();
    std::default_random_engine generator;
    double sum = 0;
    for (int i = 0; i < 256; i++) {
      // if (lambdas[i].label > 0) {
      //   std::normal_distribution<double> distribution(lambdas[i].label, 200 * sqrt(TAU));
      //   lambdas[i].label = distribution(generator);
      //   if (lambdas[i].label <= 1) {
      //     lambdas[i].label = 1;
      //   }
      //   if (lambdas[i].label > 1000) {
      //     lambdas[i].label = 1000;
      //   }
      // }
      int labelTau = lambdas[i].label * TAU;
      lambdas[i].score = lambdas[i].prob * (pow(labelTau, numberOfPacketsInTick) / factorial(numberOfPacketsInTick)) * exp(-labelTau);
      sum += lambdas[i].score;
    }
    for (int i = 0; i < 256; i++) {
      lambdas[i].prob = lambdas[i].score / sum;
    }
    qsort(lambdas, 256, sizeof(lambdaEntry), sortByProb_desc);
    for (int i = 0; i < 256; i++) {
      if (lambdas[i].prob != lambdas[i].prob) {
        exit(0);
      }
      cout << i << ": (" << lambdas[i].label << ", " << lambdas[i].prob << ")" << endl;
    }
    cout << endl << endl;
    cout << "Number of packets in tick: " << numberOfPacketsInTick << ". Lambda with highest probability: " << lambdas[0].label << ". Probability: " << lambdas[0].prob << ". Queue occupancy at " << queue_occupancy_est() << "." << endl;
    // the_window_size = ((lambdas[0].label / 10.0) - queue_occupancy_est());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{ 
  /* Default: take no action */
  num_packets_received++;
  num_packets_tick_mutex.lock();
  num_packets_tick_start++;
  num_packets_tick_mutex.unlock();

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }

}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 200; /* timeout of one second */
}
