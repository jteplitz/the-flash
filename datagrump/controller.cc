#include <limits>
#include <iostream>
#include "controller.hh"
#include "timestamp.hh"
// bandwidth window in ms
#define START_WINDOW_SIZE 1
#define BANDWIDTH_WINDOW 200
#define MIN_VEC_SIZE 10
#define THRESHOLD 4
#define RTT_THRESH 300
#define ALPHA 0.1
#define GRADIENT_THRESHOLD -1.5
#define MULT_DECREASE_COUNT 3

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
    receive_packets_(),
    send_packets_(),
    curr_window_(START_WINDOW_SIZE),
    prev_bandwidth_(0),
    bandwidth_diff_(0),
    prev_ack_time_(0),
    mult_decrease_counter(0)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << curr_window_ << endl;
  }

  return max((int) curr_window_, 1);
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */
  update_bandwidth_estimate_(send_packets_, send_timestamp);

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

void Controller::update_window_(const double bandwidth_gradient)
{
  auto receive_bandwidth = get_bandwidth_estimate_(receive_packets_);
  auto send_bandwidth = get_bandwidth_estimate_(send_packets_);
  if (receive_bandwidth < 0) {
    curr_window_ = window_size() / 2;
    return;
  }
  if (send_bandwidth > receive_bandwidth + THRESHOLD) {
    if (bandwidth_gradient < GRADIENT_THRESHOLD) {
      mult_decrease_counter++;
      if (mult_decrease_counter == MULT_DECREASE_COUNT) {
        curr_window_ = window_size() / 2;
        mult_decrease_counter = 0;
      }
    } else {
      curr_window_ -= 1.0 / window_size();
    }
  } else {
    mult_decrease_counter = 0;
    curr_window_ += 1.0 / window_size();
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
  //auto rtt = timestamp_ack_received - send_timestamp_acked;
  update_bandwidth_estimate_(receive_packets_, recv_timestamp_acked);
  auto receive_estimate = get_bandwidth_estimate_(receive_packets_);
  double gradient = 0.0;
  if (prev_bandwidth_ != 0) {
    auto delta = timestamp_ack_received - prev_ack_time_;
    if (delta != 0) {
      gradient = gradient_(receive_estimate, prev_bandwidth_, bandwidth_diff_, delta);
      //cout << timestamp_ack_received << ',' << gradient << ',' << receive_estimate << endl;
    }
  }
  prev_ack_time_ = timestamp_ack_received;
  prev_bandwidth_ = get_bandwidth_estimate_(receive_packets_);
  update_window_(gradient);
  /*if (rtt > RTT_THRESH) {
    curr_window_ = window_size() * 0.75;*
  } else {
    update_window_();
  }*/

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
  return 1000; /* timeout of one second */
}

void Controller::update_bandwidth_estimate_(vector<uint64_t> &vec, uint64_t time)
{
  vec.push_back(time);
  if (vec.size() > MIN_VEC_SIZE) {
    auto min_time = time - BANDWIDTH_WINDOW;
    vector<uint64_t>::iterator i = vec.begin();
    if (vec[0] < min_time ) {
      for(; i != vec.end(); i++) {
        if (*i >= min_time) {
          break;
        }
      }
    }
    vec.erase(vec.begin(), i);
  }
}

/**
 * Returns the bandwidth estimate in packets per second.
 */
double Controller::get_bandwidth_estimate_(vector<uint64_t> &vec)
{
  auto start_time = vec[0];
  auto end_time = vec[vec.size() - 1];
  auto estimate = vec.size() / ((double) (end_time - start_time) / 1000);
  return estimate;
}

double Controller::gradient_( const double curr, double &prev, double &diff,
                              uint64_t delta)
{
  if (prev == numeric_limits<double>::infinity() || 
      curr == numeric_limits<double>::infinity()) {
    return 0.0;
  }
  double new_diff = (curr - prev) / delta;
  prev = curr;
  diff = (1.0 - ALPHA) * diff + ALPHA * new_diff;
  return diff;
}
