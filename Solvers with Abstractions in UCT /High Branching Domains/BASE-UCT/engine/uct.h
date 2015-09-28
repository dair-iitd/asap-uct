/*
 *  Copyright (C) 2011 Universidad Simon Bolivar
 * 
 *  Permission is hereby granted to distribute this software for
 *  non-commercial research purposes, provided that this copyright
 *  notice is included with any such distribution.
 *  
 *  THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 *  EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
 *  SOFTWARE IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU
 *  ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 *  
 *  Blai Bonet, bonet@ldc.usb.ve
 *
 */

#ifndef UCT_H
#define UCT_H

#include "policy.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>
#include <ctime>

//#define DEBUG

extern bool new_trial;
extern  bool goal_reached;
 extern int this_trial_decisions;
 extern int count_goal_updates;
 extern float decrease_in_goal_distance;



namespace Online {

namespace Policy {

namespace UCT {

////////////////////////////////////////////////
//
// Tree
//

template<typename T> struct node_t {
    mutable std::vector<unsigned> counts_;
    mutable std::vector<float> values_;
    node_t(int num_actions)
      : counts_(1+num_actions, 0),
        values_(1+num_actions, 0) {
    } 
    ~node_t() { }
};

////////////////////////////////////////////////
//
// Hash Table
//

template<typename T> struct map_functions_t {
    size_t operator()(const std::pair<unsigned, T> &p) const {
        return p.second.hash();
    }
};

struct data_t {
    std::vector<float> values_;
    std::vector<int> counts_;
    data_t(const std::vector<float> &values, const std::vector<int> &counts)
      : values_(values), counts_(counts) { }
    data_t(const data_t &data)
      : values_(data.values_), counts_(data.counts_) { }
#if 0
    data_t(data_t &&data)
      : values_(std::move(data.values_)), counts_(std::move(data.counts_)) { }
#endif
};

template<typename T> class hash_t :
  public Hash::generic_hash_map_t<std::pair<unsigned, T>,
                                  data_t,
                                  map_functions_t<T> > {
  public:
    typedef typename Hash::generic_hash_map_t<std::pair<unsigned, T>,
                                              data_t,
                                              map_functions_t<T> >
            base_type;
    typedef typename base_type::const_iterator const_iterator;
    const_iterator begin() const { return base_type::begin(); }
    const_iterator end() const { return base_type::end(); }

  public:
    hash_t() { }
    virtual ~hash_t() { }
    void print(std::ostream &os) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            os << "(" << it->first.first << "," << it->first.second << ")" << std::endl;
        }
    }
};

////////////////////////////////////////////////
//
// Policy
//

template<typename T> class uct_t : public improvement_t<T> {
  using policy_t<T>::problem;

  protected:
    unsigned width_;
    mutable unsigned horizon_;
    float parameter_;
    bool random_ties_;
    mutable hash_t<T> table_;
    //mutable double timestore[100]={0};
    mutable float rem_time;
    mutable float total_time;

  public:
    uct_t(const policy_t<T> &base_policy,
          unsigned width,
          unsigned horizon,
          float parameter,
          bool random_ties)
      : improvement_t<T>(base_policy),
        width_(width),
        horizon_(horizon),
        parameter_(parameter),
        random_ties_(random_ties) {
        std::stringstream name_stream;
        name_stream << "uct("
                    << "width=" << width_
                    << ",horizon=" << horizon_
                    << ",par=" << parameter_
                    << ",random-ties=" << (random_ties_ ? "true" : "false")
                    << ")";
        policy_t<T>::set_name(name_stream.str());
    }
    virtual ~uct_t() { }

    virtual Problem::action_t operator()(const T &s) const {
        ++policy_t<T>::decisions_;
        table_.clear();

       /*  total_time=parameter_;
        float this_decision_time;
        unsigned this_decision_width;
        int rem_decisions;
        bool adapt=goal_reached;

        if(new_trial)
        {
            rem_time=total_time;
            new_trial=false;
            this_trial_decisions=1;
        }
        else
            ++this_trial_decisions;
       //  if(horizon_ +this_trial_decisions >=41)
         //   horizon_=41-this_trial_decisions;
      //decrease_in_goal_distance=0;
        //adapt=false;
        rem_decisions=101-this_trial_decisions-(int)(decrease_in_goal_distance+0.5);
        if(rem_decisions<5)
            rem_decisions=5;
        decrease_in_goal_distance=0;
       count_goal_updates=0;
       goal_reached=false;
        //policy_t<T>::decrease_in_goal_distance=0;
       // policy_t<T>::count_goal_updates=0;
       // policy_t<T>::goal_reached=false;
        //std::cout<<"Rem Decisions"<<rem_decisions<<"\n";
         //std::cout<<"Goal distance reduction"<<decrease_in_goal_distance<<"\n";
        if(adapt)
        {
          //std::cout<<"Goal reached"<<rem_decisions<<"\n";
            this_decision_time=rem_time*2/(rem_decisions+1);
        }
        else
        {
           // std::cout<<"\nremining Decisions"<<rem_decisions<<"\n";
            this_decision_time=rem_time/rem_decisions;
            // std::cout<<"\nDecisions"<<this_decision_time;
        } 
        if(this_decision_time>500)
            this_decision_time=500;
        //nvaigation
       //this_decision_width=this_decision_time*48+20;
        //sailing
        //this_decision_width=this_decision_time*90+2;
        //sysadmin
        //this_decision_width=this_decision_time*13.04;
       //Sailing_Instance2
       //this_decision_width=this_decision_time*93+3;
        //Sysadmin-2
       //this_decision_width=this_decision_time*14+5;
       //Navigation-2
       // this_decision_width=this_decision_time*55.5+3;
       //sysadmin-1
       this_decision_width=this_decision_time*0.973+0.4;
      this_decision_width=parameter_;
       // unsigned l=width_;





        clock_t start, end;
        double cpu_time_used;
        //start=clock();
        double start_time,end_time;
        
        start_time= Utils::my_read_time_in_milli_seconds();
        //end_time= Utils::my_read_time_in_milli_seconds();
        //while (end_time-start_time<this_decision_time){*/
        for( unsigned i = 0; i < parameter_; ++i ){
            search_tree(s, 0);
            //end_time= Utils::my_read_time_in_milli_seconds();
        }
      //  rem_time-=(this_decision_time);
        //end_time= Utils::my_read_time_in_milli_seconds();
        //std::cout<<","<<std::setprecision(2)<<end_time-start_time;
       // end=clock();
        //cpu_time_used=((double) (end - start)) / CLOCKS_PER_SEC;
    //  std::cout<<"\n,"<<end_time-start_time<<" " <<this_decision_width;
        //std::cout<<end_time-start_time<<"\n";
       //   std::cout<<","<<std::setprecision(4)<<cpu_time_used;
      
        typename hash_t<T>::iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        Problem::action_t action = select_action(s, it->second, 0, false, random_ties_);
        assert(problem().applicable(s, action));
       // std::cout<<"\nAction-"<<action<<" State"<<s<<"\n";

        return action;
    }
    virtual const policy_t<T>* clone() const {
        return new uct_t(improvement_t<T>::base_policy_, width_, horizon_, parameter_, random_ties_);
    }
    virtual void print_stats(std::ostream &os) const {
        os << "stats: policy=" << policy_t<T>::name() << std::endl;
        os << "stats: decisions=" << policy_t<T>::decisions_ << std::endl;
        improvement_t<T>::base_policy_.print_stats(os);
    }

    float value(const T &s, Problem::action_t a) const {
        typename hash_t<T>::const_iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        return it->second.values_[1+a];
    }
    unsigned count(const T &s, Problem::action_t a) const {
        typename hash_t<T>::const_iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        return it->second.counts_[1+a];
    }
    size_t size() const { return table_.size(); }
    void print_table(std::ostream &os) const {
        table_.print(os);
    }

    float search_tree(const T &s, unsigned depth) const {
#ifdef DEBUG
        std::cout << std::setw(2*depth) << "" << "search_tree(" << s << "):";
#endif

        // if(problem().terminal(s))
        // {
        //     policy_t<T>::goal_reached=true;
        // }
         if( (depth == horizon_) || problem().terminal(s) ){//|| (this_trial_decisions+depth ==100)) {
        //if( (depth == horizon_) || problem().terminal(s) ) {
#ifdef DEBUG
            std::cout << " end" << std::endl;
#endif
            return 0;
        }

        if( problem().dead_end(s) ) {
#ifdef DEBUG
            std::cout << " dead-end" << std::endl;
#endif
            return problem().dead_end_value();
        }

        typename hash_t<T>::iterator it = table_.find(std::make_pair(depth, s));

        if( it == table_.end() ) {
            std::vector<float> values(1 + problem().number_actions(s), 0);
            std::vector<int> counts(1 + problem().number_actions(s), 0);
            table_.insert(std::make_pair(std::make_pair(depth, s), data_t(values, counts)));
            float value = evaluate(s, depth);
#ifdef DEBUG
            std::cout << " insert in tree w/ value=" << value << std::endl;
#endif
            return value;
        } else {
            // select action for this node and increase counts
            Problem::action_t a = select_action(s, it->second, depth, true, random_ties_);
           // std::cout<<"Type-2 Select Act:"<<a<<"\n";
            ++it->second.counts_[0];
            ++it->second.counts_[1+a];

            // sample next state
            //std::pair<const T, bool> p = problem().sample(s, a);
              std:: pair<T,bool> p;
             problem().sample_factored(s,a,p.first);
             //std::cout<<"\n"<<p.first;
                     float cost = problem().cost(s, a);
            //std::cout<<"Cost-"<<s<<" Action"<<a<<" " <<cost<<"\n";

#ifdef DEBUG
            std::cout << " count=" << it->second.counts_[0]-1
                      << " fetch " << std::setprecision(5) << it->second.values_[1+a]
                      << " a=" << a
                      << " next=" << p.first
                      << std::endl;
#endif

            // do recursion and update value
            float &old_value = it->second.values_[1+a];
            
            int n = it->second.counts_[1+a];
            float new_value = cost +
              problem().discount() * search_tree(p.first, 1 + depth);
            old_value += (new_value - old_value) / n;
             
            return old_value;
        }
    }

    Problem::action_t select_action(const T &state,
                                    const data_t &data,
                                    int depth,
                                    bool add_bonus,
                                    bool random_ties) const {
        float log_ns = logf(data.counts_[0]);


       
        std::vector<Problem::action_t> best_actions;
        int nactions = problem().number_actions(state);
        float best_value = std::numeric_limits<float>::max();

        best_actions.reserve(random_ties ? nactions : 1);
        for( Problem::action_t a = 0; a < nactions; ++a ) {
           // std::cout<<"State Here "<<state<<"Action "<<a <<" "<<problem().applicable(state, a);
            if( problem().applicable(state, a) ) {
                // if this action has never been taken in this node, select it
                if( data.counts_[1+a] == 0 ) {
                    return a;
                }
                
                // compute score of action adding bonus (if applicable)
                assert(data.counts_[0] > 0);
               // float par = parameter_ == 0 ? -data.values_[1+a] : parameter_;
                float par = -fabs(data.values_[1+a]);
                float bonus = add_bonus ? par * sqrtf(2 * log_ns / data.counts_[1+a]) : 0;
                float value = data.values_[1+a] + bonus;
               //  std::cout<<"in Action:"<<a<<"Value"<<std::setprecision(5)<<value<<"\n";
                // update best action so far
                if( value <= best_value ) {
                  
                    if( value < best_value ) {

                       // std::cout<<"best7:"<<best_value<<"\n";  

                         //std::cout<<"value"<<value<<"\n"; 
                        best_value = value;
                                         
                        best_actions.clear();
                    }
                    if( random_ties || best_actions.empty() )
                    {
                      // std::cout<<"best2 action:"<<a<<" para:"<<par;
                        best_actions.push_back(a);
                    }
                }
            }
        }
    // std::cout<<" best value:"<<best_value<<" best Action:"<<best_actions[0]<<"\n";
        assert(!best_actions.empty());
        return best_actions[Random::uniform(best_actions.size())];
    }

    float evaluate(const T &s, unsigned depth) const {
        return Evaluation::evaluation(improvement_t<T>::base_policy_,
                                      s, 1, horizon_ - depth);
    }
};

}; // namespace UCT

template<typename T>
inline const policy_t<T>* make_uct(const policy_t<T> &base_policy,
                                   unsigned width,
                                   unsigned horizon,
                                   float parameter,
                                   bool random_ties) {
    return new UCT::uct_t<T>(base_policy, width, horizon, parameter, random_ties);
}

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

