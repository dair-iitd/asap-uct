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
  ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
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
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>
#include <map>
#include <array>
#include <unistd.h>
 #include <ctime>
 #include <unordered_map>

//#define DEBUG
//#define MY_DEBUG

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
    

    /* SAU declarations */
    typedef std::pair<unsigned, T> tree_node_; 
    typedef std::pair<tree_node_,Problem::action_t> state_action_pair_; 
    mutable std::map<tree_node_, std::vector<tree_node_>> abstract_to_ground_; // Mapping from abstract states to set of ground states
    mutable std::map<state_action_pair_,std::vector<state_action_pair_>> SA_abstract_to_ground_; // Mapping from abstract (state, action) pair to set of ground (state, action) pairs
    mutable std::map<state_action_pair_,state_action_pair_> inverse_SA_; // Inverse mapping from ground (state, action) pair to abstract (state, action) pair
    mutable std::map<state_action_pair_,std::pair<int,float>> SA_abstract_data_;// Data Corresponding to an abstract node
    mutable std::map <T,T> temp_inverse_state_map, temp_inverse_undersampled_state_map;  
    //mutable int Count1=0;
    mutable float rem_time;
    mutable float total_time;
    mutable std::vector<state_action_pair_> temp_abs_SA;
    
    typedef std::map<T,float> innerMap;
    mutable std::map<state_action_pair_,innerMap> SuperMap;
    mutable std::map<innerMap,std::vector<state_action_pair_>> revSuperVMap;
    typedef std::map<state_action_pair_,int> innerStateMap;
    mutable std::map<innerStateMap,T> revStateSuperMap;
    //mutable std::unordered_map<int,int> m1;


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

        

        //Count1++;
       
        table_.clear();
      
        unsigned l=width_;

        /*total_time=parameter_;
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
        //if(horizon_ +this_trial_decisions >=41)
         //  horizon_=41-this_trial_decisions;
      //  decrease_in_goal_distance=0;
       // adapt=false;
        //std::cout<<"Decrease-"<<decrease_in_goal_distance<<"\n";
        rem_decisions=101-this_trial_decisions-(int)(decrease_in_goal_distance+0.5);
        if(rem_decisions<5)
            rem_decisions=5;
        decrease_in_goal_distance=0;
       count_goal_updates=0;
       goal_reached=false;
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
        //std::cout<<"this_decision_time"<<this_decision_time<<"\n";
        //this_decision_width=calculate_width(this_decision_time);
        /////////////////Navigaton
        //this_decision_width=this_decision_time*17.8+130;
        ////////////////Sailing 
         //this_decision_width=this_decision_time*52+45;
        //this_decision_width=12*this_decision_time+5;
       //Sailing 2- 20- 50
       //this_decision_width=66*this_decision_time+20;
      // Not used  this_decision_width=23*this_decision_time+25;
        //sysadmin-2
        //this_decision_width=16*this_decision_time;
        //navigation-2
        //this_decision_width=38*this_decision_time+10;
        //sysadmin 1
        this_decision_width=0.285*this_decision_time+0.19;
        rem_time-=this_decision_time;
       this_decision_width=parameter_;
        unsigned l=width_;
        double start_time,end_time;
        start_time= Utils::my_read_time_in_milli_seconds();
        //std::cout<<"START TIME:"<<start_time<<"\n";
                 
       */
        /*abstract_to_ground_.clear();          
                SA_abstract_to_ground_.clear();
                inverse_SA_.clear();
                SA_abstract_data_.clear();
                temp_inverse_state_map.clear();
                temp_inverse_undersampled_state_map.clear();*/
                unsigned this_decision_time=parameter_;
                unsigned this_decision_width=parameter_;
               /* if(l==2)
                    this_decision_width=this_decision_time*0.67+41;
                else
                    this_decision_width=this_decision_time*0.88+41;*/
        double start_time,end_time;
       for (unsigned m = 0; m < l; ++m) {             
            //start_time= Utils::my_read_time_in_milli_seconds();
           for( unsigned i = 0; i <this_decision_width/l; ++i ) {
            search_tree(s, 0);
            //end_time= Utils::my_read_time_in_milli_seconds();
      }
                // end_time= Utils::my_read_time_in_milli_seconds();
                //std::cout<<","<<end_time-start_time<<" Tree Buiding Time"<<"\n";
      			//
      			//std::cout<<"\n,"<<end_time-start_time;
                update_original_nodes();
                abstract_to_ground_.clear();          
                SA_abstract_to_ground_.clear();
                inverse_SA_.clear();
                SA_abstract_data_.clear();
                temp_inverse_state_map.clear();
                temp_inverse_undersampled_state_map.clear();
                //start_time= Utils::my_read_time_in_milli_seconds();
                if (m!=l-1)
                {
        	        construct_abstract_tree(s);
                    //end_time= Utils::my_read_time_in_milli_seconds();
                 //   std::cout<<","<<end_time-start_time<<"  Non Updation Time\n";
        	        update_data_values();
                } 
                //end_time= Utils::my_read_time_in_milli_seconds();
              //  std::cout<<","<<end_time-start_time<<"  Abstraction Calculation Time\n";
        }
           
       // end_time= Utils::my_read_time_in_milli_seconds();
       
        //std::cout<<"END TIME:"<<end_time<<"\n";
     	//std::cout<<","<<end_time-start_time<<"  "<< this_decision_width<<"\n";
          //std::cout<<end_time-start_time<<"\n";

        typename hash_t<T>::iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
    
        Problem::action_t action = select_action(s, it->second, 0, false, random_ties_);
       // std::cout<<"\n"<<s<<"  " <<action;
        assert(problem().applicable(s, action));
     	//exit(0);
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

    /*SAU Checks if a state has an unsampled action */
    bool isUndersampledState(const T &s, unsigned depth) const{
        typename hash_t<T>::iterator it = table_.find(std::make_pair(depth, s));
        int num_applicable_actions=0;
        int nactions = problem().number_actions(s);
        Problem::action_t a;
        for(a = 0; a < nactions; ++a ) {
            if( problem().applicable(s, a)) 
                num_applicable_actions++;
        }
        //Code Modiefied SAU
        //if(it->second.counts_[0]>=num_applicable_actions)
        if(getTotalCount(s, depth, it->second.counts_)>=num_applicable_actions)
            return false;

        return true;

    }

    // /* Takes the average count before updating abstraction */
    // void update_equi_SA_nodes()const{
    //     typename hash_t<T>::iterator it;
    //     typename std::map<state_action_pair_,std::vector<state_action_pair_>>::iterator actit;
    //     typename std::vector<state_action_pair_>::iterator vec_act_it;
    //     typename std::map<state_action_pair_,std::pair<int,float>>::iterator data_it;
    //      // Calculating average
    //     for(actit=SA_abstract_to_ground_.begin();actit!=SA_abstract_to_ground_.end();++actit)
    //     {
    //         data_it=SA_abstract_data_.find(actit->first);
    //         unsigned abstractionSize=actit->second.size();
    //         for(vec_act_it=actit->second.begin();vec_act_it!=actit->second.end();++vec_act_it)
    //         {
    //             it=table_.find(vec_act_it->first);
    //             Problem:: action_t current_node_action=vec_act_it->second;
    //             it->second.counts_[1+current_node_action]=data_it->second.first/abstractionSize;
    //         }
    //     }    

    // }

bool areStatesEquivalent (const T &s1, const T &s2, unsigned depth)const{
    int nactions=problem().number_actions(s1);
    
    std::map<state_action_pair_,int> match;
    state_action_pair_ SA1,SA2;
    typename std::map<state_action_pair_,int>::iterator matchIt;
    for(unsigned i=0;i<nactions;i++)
    {
        if(problem().applicable(s1,i))
        {
            SA1=std::make_pair(std::make_pair(depth,s1),i);
            state_action_pair_ absPair=inverse_SA_[SA1];
            matchIt=match.find(absPair);
            if(matchIt!=match.end())
                matchIt->second++;
            else
                match.insert(std::make_pair(absPair,1));
        }
    }
    for(unsigned i=0;i<nactions;i++)
    {
        if(problem().applicable(s2,i))
        {
            SA2=std::make_pair(std::make_pair(depth,s2),i);
            state_action_pair_ absPair=inverse_SA_[SA2];
            matchIt=match.find(absPair);
            if(matchIt!=match.end())
                if(matchIt->second==1)
                    match.erase(matchIt);
                else 
                    matchIt->second--;
            else
                return false;
        }
    }
    if(match.empty())
        return true;
    else
        return false;

}

   /* bool areStatesEquivalent(const T &s1,const T &s2, unsigned depth)const{
        int nactions=problem().number_actions(s1);
        state_action_pair_ SA1,SA2;
        bool flag_match[nactions];

        for(Problem::action_t a=0;a<nactions;a++)
            flag_match[a]=false;

        for(Problem::action_t a1=0;a1<nactions;a1++)
        {
            bool flag_found=false;  
                if(problem().applicable(s1,a1))
                {    
                    SA1=std::make_pair(std::make_pair(depth,s1),a1);
                    for(Problem::action_t a2=0;a2<nactions;a2++)
                    {
                        if(problem().applicable(s2,a2))
                        {    
                            SA2=std::make_pair(std::make_pair(depth,s2),a2); 
                            if((inverse_SA_.count(SA1)==0)||(inverse_SA_.count(SA2)==0))
                            {
                                std::cout<<" \n Inverse Map not Found";
                                exit(0);
                            }
                            if(inverse_SA_[SA1]==inverse_SA_[SA2])
                            {
                                flag_match[a2]=true;
                                flag_found=true;
                            }
                        }
                        else
                            flag_match[a2]=true;
                    }
                    if(!flag_found)
                        return false;        
                }
        }
        for(Problem::action_t a=0;a<nactions;a++)
        {
            if(!flag_match[a])
                return false;
        }   

        return true;
    }*/

    void update_inverse_state_map( unsigned currentDepth) const {
    	unsigned abstractDepth; T abstractState;
    	typename std::map <tree_node_,std::vector<tree_node_> >::reverse_iterator rgit;
    	for (rgit = abstract_to_ground_.rbegin(); rgit != abstract_to_ground_.rend(); ++rgit)
            {
                abstractDepth=(rgit->first).first;
                abstractState=(rgit->first).second;
                if (abstractDepth==currentDepth+1){
                    std::vector<tree_node_> nextDepthVector=rgit->second;
                    for (unsigned j=0; j< nextDepthVector.size(); j++)
                        temp_inverse_state_map.insert(std::pair<T,T>(nextDepthVector[j].second,abstractState));
                    }
                else if (abstractDepth<currentDepth+1)
                    break;
            }
    }

    bool isEquivalentSA( const T &FirstState, int FirstAction, const T &SecondState, int SecondAction) const {

        if (problem().cost(FirstState,FirstAction)!=problem().cost(SecondState,SecondAction))
            return false;

        typename std::map <T,T>::iterator temp_map_it;
        //std::vector<std::pair<T, float> > outcomesFirst, outcomesSecond;
        //problem().next(FirstState, FirstAction, outcomesFirst);
        //problem().next(SecondState, SecondAction, outcomesSecond);
        //unsigned osizeFirst = outcomesFirst.size(), osizeSecond = outcomesSecond.size();
        std::map <T,std::array<float,2>> transComparison;
        typename std::map <T,std::array<float,2>>::iterator transIter;
        std::array<float,2> transTemp;
        transTemp[0]=0; //Sau_Ankit
        transTemp[1]=0; //Sau_Ankit
        T temp_outcome_node;
        float transition_prob;



        for(temp_map_it=temp_inverse_state_map.begin();temp_map_it!=temp_inverse_state_map.end();temp_map_it++)
        {
            temp_outcome_node=temp_map_it->second;
            transition_prob=problem().calculate_transition(FirstState,temp_map_it->first,FirstAction);
            transIter=transComparison.find(temp_outcome_node);
            //std::cout<<"Prob Value "<<transition_prob<<"\n";
            if (transIter==transComparison.end()){
                transTemp[0]=transition_prob;
                transComparison.insert(std::pair<T,std::array<float,2>>(temp_outcome_node,transTemp));
            } else {
                transTemp=transIter->second;
                transTemp[0]=transTemp[0]+transition_prob;
                transIter->second=transTemp;
            }
        }
        /*std::cout<<"\n Map1"<<FirstState<<" "<<FirstAction<<"\n";
        //return temp_trans_map;
        for(transIter=transComparison.begin();transIter!=transComparison.end();transIter++)
        {
            std::cout<<transIter->first<<" "<< transIter->second[0]<"\n";
        }*/
         for(temp_map_it=temp_inverse_state_map.begin();temp_map_it!=temp_inverse_state_map.end();temp_map_it++)
        {
            temp_outcome_node=temp_map_it->second;
            transition_prob=problem().calculate_transition(SecondState,temp_map_it->first,SecondAction);
            transIter=transComparison.find(temp_outcome_node);
            if (transIter==transComparison.end()){
                return false;
            } else {
                transTemp=transIter->second;
                transTemp[1]=transTemp[1]+transition_prob;
                transIter->second=transTemp;
            }
        }



/*
        for (unsigned o=0; o<osizeFirst; o++)
        {
           if(outcomesFirst[o].second==0) 
               continue;
            if(temp_inverse_state_map.find(outcomesFirst[o].first)!=temp_inverse_state_map.end())
                temp_outcome_node=temp_inverse_state_map[outcomesFirst[o].first];
             else
                continue;             	
                 //temp_outcome_node=outcomesFirst[o].first;
            transIter=transComparison.find(temp_outcome_node);
            if (transIter==transComparison.end()){
                transTemp[0]=outcomesFirst[o].second;
                transComparison.insert(std::pair<T,std::array<float,2>>(temp_outcome_node,transTemp));
            } else {
                transTemp=transIter->second;
                transTemp[0]=transTemp[0]+outcomesFirst[o].second;
                transIter->second=transTemp;
            }
        }

        for (unsigned o=0; o<osizeSecond; o++)
        {
            if(outcomesSecond[o].second==0) 
               continue;
            if(temp_inverse_state_map.find(outcomesSecond[o].first)!=temp_inverse_state_map.end())
                temp_outcome_node=temp_inverse_state_map[outcomesSecond[o].first];
            else
            	continue;
                //temp_outcome_node=outcomesSecond[o].first;
            transIter=transComparison.find(temp_outcome_node);
            if (transIter==transComparison.end()){
                return false;
            } else {
                transTemp=transIter->second;
                transTemp[1]=transTemp[1]+outcomesSecond[o].second;
                transIter->second=transTemp;
            }
        }*/


        for (transIter=transComparison.begin(); transIter!=transComparison.end(); ++transIter)
        {
            transTemp=transIter->second;
            if (transTemp[0]!=transTemp[1]){
                return false;
            }
        }
        return true;
    }
    
    bool isEquivalentSA1(state_action_pair_ currentSApair, state_action_pair_ abstractSApair)const{
        
        if (problem().cost(currentSApair.first.second,currentSApair.second)!=problem().cost(abstractSApair.first.second,abstractSApair.second))
            return false;

        std::map<T,float> Map1;
        std::map<T,float> Map2;
        
        
       
        Map1=SuperMap[currentSApair];
        Map2=SuperMap[abstractSApair];
        typename std::map<T,float> ::iterator m1;
        for(m1=Map1.begin();m1!=Map1.end();m1++)
        {
            if(Map2.find(m1->first)!=Map2.end())
            {    
                if(fabs(Map2[m1->first]-(m1->second))>0.01)
                {
                    //std::cout<<"here\n"<<Map2[m1->first]<<" "<<m1->second<<"\n";
                    //exit(0);
                    return false;

                }
                else
                {
                    //std::cout<<"kjhk\n"<<Map2[m1->first]<<" "<<m1->second<<" "<<abs(Map2[m1->first]-(m1->second))<<"\n";
                    //exit(0);
                    Map2.erase(m1->first);
                }
            }
            else
                return false;    
        }
        if(Map2.empty())
            return true;
        else
            return false;
        /*if(Map1==Map2)
            return true;
        else
            return false;*/
    }
    state_action_pair_ get_equivalent_abstractSApair1( state_action_pair_ currentSApair) const {
     /*   std::map<T,float> temp_trans_map;
        //temp_trans_map.clear();
        typename std::map<T,float>::iterator transIter;
        float transition_prob;
        T temp_outcome_node;
        
        typename std::map <T,T>::iterator temp_map_it;
        for(temp_map_it=temp_inverse_state_map.begin();temp_map_it!=temp_inverse_state_map.end();temp_map_it++)
        {
            temp_outcome_node=temp_map_it->second;
            transition_prob=problem().calculate_transition(currentSApair.first.second,temp_map_it->first,currentSApair.second);
            transIter=temp_trans_map.find(temp_outcome_node);
           //std::cout<<"Transi"<<transition_prob<<"\n";
            if (transIter==temp_trans_map.end()){
                temp_trans_map.insert(std::pair<T,float>(temp_outcome_node,transition_prob));
            } 
            else {
                transIter->second+=transition_prob;
            }
        }*/


        innerMap currentSApairMap=SuperMap[currentSApair];
        typename std::map<innerMap,std::vector<state_action_pair_>>::iterator itRevSuperMap;
        itRevSuperMap=revSuperVMap.find(currentSApairMap);
        state_action_pair_ abstractSApair;
        if(itRevSuperMap==revSuperVMap.end())
        {
            std::vector<state_action_pair_> tempVector;
            tempVector.push_back(currentSApair);
            revSuperVMap.insert(std::make_pair(currentSApairMap,tempVector));

        }
        else
        {
             std::vector<state_action_pair_> tempVector=itRevSuperMap->second;
             for(unsigned i=0;i<tempVector.size();i++)
             {
                abstractSApair=tempVector[i];
                if (problem().cost(currentSApair.first.second,currentSApair.second)==problem().cost(abstractSApair.first.second,abstractSApair.second))
                    return abstractSApair;
             }
             itRevSuperMap->second.push_back(currentSApair);

        }
        return currentSApair;
    }


    

    T get_equivalent_abstractState(const T &s, unsigned depth)const{
            int nactions=problem().number_actions(s);
            innerStateMap tempMap;
            typename innerStateMap::iterator itInnerStateMap;
            state_action_pair_ SA1;
            state_action_pair_ absPair;
            for(unsigned a=0;a<nactions;a++)
            {
                if(problem().applicable(s,a))
                {
                    SA1=std::make_pair(std::make_pair(depth,s),a);
                    absPair=inverse_SA_[SA1];
                    itInnerStateMap=tempMap.find(absPair);
                    if(itInnerStateMap!=tempMap.end())
                        itInnerStateMap->second+=1;
                    else
                        tempMap.insert(std::make_pair(absPair,1));

                }
            }
            typename std::map<innerStateMap,T>::iterator tempIt;
            tempIt=revStateSuperMap.find(tempMap);
            if(tempIt!=revStateSuperMap.end())
                return tempIt->second;
            else
                revStateSuperMap.insert(std::make_pair(tempMap,s));
            return s;

    }


    state_action_pair_ get_equivalent_abstractSApair( state_action_pair_ currentSApair) const {
     	unsigned currentDepth=(currentSApair.first).first;
        T currentState=(currentSApair.first).second;
        Problem::action_t currentAction = currentSApair.second;   
        bool isEquiv=false;     
        typename std::map <T,T>::iterator invit;
        typename std::map <state_action_pair_,std::vector<state_action_pair_>>::reverse_iterator action_map_it;
        unsigned abstractDepth;T abstractState; state_action_pair_ abstractSApair;
        //typename std::vector<state_action_pair_>::iterator tempAbstractIt;
        for(unsigned i=0;i<temp_abs_SA.size();i++){
            //std::cout<<"inside Get Equivalent\n";
            abstractSApair=temp_abs_SA[i];
     	//for(action_map_it=SA_abstract_to_ground_.rbegin();action_map_it!=SA_abstract_to_ground_.rend();++action_map_it){
          //  abstractSApair=action_map_it->first;
            abstractDepth=abstractSApair.first.first;
            abstractState=abstractSApair.first.second;
            //std::cout<<"Depth is "<<abstractDepth<<"\n";
            Problem::action_t abstract_action=abstractSApair.second;

         //if(currentDepth>abstractDepth)
           //     break;
            //if(currentDepth==abstractDepth){
                bool flagNotEquiv=isEquivalentSA1(currentSApair,abstractSApair);
                //bool flagNotEquiv=isEquivalentSA(currentState,currentAction,abstractState, abstract_action);
                
                if(!flagNotEquiv)
                   continue;
               isEquiv=true;
               break;

            //}  
        }

        if (isEquiv)
        	return abstractSApair;
        else
        	return currentSApair;
     }
     void calculate_abs_trans_probs(state_action_pair_ currentSApair) const{
     
        std::map<T,float> temp_trans_map;
        temp_trans_map.clear();
        typename std::map<T,float>::iterator transIter;
        float transition_prob;
        T temp_outcome_node;
        
        typename std::map <T,T>::iterator temp_map_it;
        for(temp_map_it=temp_inverse_state_map.begin();temp_map_it!=temp_inverse_state_map.end();temp_map_it++)
        {
            temp_outcome_node=temp_map_it->second;
            transition_prob=problem().calculate_transition(currentSApair.first.second,temp_map_it->first,currentSApair.second);
            transIter=temp_trans_map.find(temp_outcome_node);
           //std::cout<<"Transi"<<transition_prob<<"\n";
            if (transIter==temp_trans_map.end()){
                temp_trans_map.insert(std::pair<T,float>(temp_outcome_node,transition_prob));
            } 
            else {
                transIter->second+=transition_prob;
            }
        }
   /*std::cout<<"\n Map"<<currentSApair.first.second<<" "<<currentSApair.second<<"\n";
       // return temp_trans_map;
        for(transIter=temp_trans_map.begin();transIter!=temp_trans_map.end();transIter++)
       {
           std::cout<<transIter->first<<" "<< transIter->second<<"\n";
        }*/
        SuperMap.insert(std::make_pair(currentSApair,temp_trans_map));
        //revSuperMMap.insert(std::make_pair(temp_trans_map,currentSApair));
     }

    /* SAU Function to compute abstractions */
    void construct_abstract_tree(const T &s) const {

                  //ground to abstract mapping stored only for previous level
        T repUndersampledNode;
        bool flagUndersampled=0;

        std::map <tree_node_, data_t> orderedMap;
        typename hash_t<T>::const_iterator it;
         typename std::map <tree_node_, data_t>::reverse_iterator rit;
         
        double start_time,end_time;
        //start_time=  Utils::my_read_time_in_milli_seconds();
        for (it = table_.begin(); it != table_.end(); ++it)
        {	
           
            
            orderedMap.insert(std::make_pair(it->first,it->second));
            
        }
          //end_time= Utils::my_read_time_in_milli_seconds();
          //std::cout<<end_time-start_time<<"\n";
          
       
        /*for ( rit= orderedMap.rbegin(); rit != orderedMap.rend(); ++rit)
        {
           ;// std::cout<<"\n"<<"Depth"<<(rit->first).first<<" state"<<(rit->first).second;
        }*/
		
		typename std::map <tree_node_,std::vector<tree_node_> >::reverse_iterator rgit;
        typename std::map <state_action_pair_,std::vector<state_action_pair_>>::iterator action_map_it;

        typename std::map <state_action_pair_,state_action_pair_>::iterator rev_action_map_it;
        unsigned oldDepth=0;
        
        for ( rit= orderedMap.rbegin(); rit != orderedMap.rend(); ++rit)
        {
           
            unsigned currentDepth=(rit->first).first;
            T currentState=(rit->first).second;
            
            if (currentDepth!=oldDepth){
                oldDepth=currentDepth;
                temp_inverse_state_map.clear();
                temp_abs_SA.clear();
                //start_time= Utils::my_read_time_in_milli_seconds();
                temp_inverse_state_map=temp_inverse_undersampled_state_map;
                update_inverse_state_map( currentDepth);
                //end_time= Utils::my_read_time_in_milli_seconds();
                //std::cout<<"\nTime here"<<end_time-start_time<<"\n";
                temp_inverse_undersampled_state_map.clear();
                flagUndersampled=0;
                SuperMap.clear();
                revSuperVMap.clear();
                revStateSuperMap.clear();
            }

            if(isUndersampledState(currentState,currentDepth)){            
                if (flagUndersampled==0) {
                    repUndersampledNode=currentState;
                    flagUndersampled=1;
                }
                temp_inverse_undersampled_state_map.insert(std::pair<T,T>(currentState,repUndersampledNode));                

            }   else { 
                typename std::map<T,T>::iterator t1;
                /*for(t1=temp_inverse_state_map.begin();t1!=temp_inverse_state_map.end();t1++)
                {
                    std::cout<<"\nState Map"<<t1->first<<" "<<t1->second; 
                }*/
               tree_node_ currentNode=rit->first;
               int nactions=problem().number_actions(currentState);
               unsigned abstractDepth;
               T abstractState; state_action_pair_ abstractSApair;
               // Building Action mapping
              // start_time=Utils::my_read_time_in_milli_seconds();
               for(Problem::action_t a=0;a<nactions;a++){

                    if(problem().applicable(currentState, a))
                    {                    
                        state_action_pair_ currentSApair(std::make_pair(currentNode,a));
                        calculate_abs_trans_probs(currentSApair);
                        //start_time=Utils::my_read_time_in_milli_seconds();

                        state_action_pair_ answer = get_equivalent_abstractSApair( currentSApair);
                        //end_time=Utils:: my_read_time_in_milli_seconds();
                        //std::cout<<"Get Equivalent Action Pair"<<end_time-start_time<<"\n";
                        action_map_it=SA_abstract_to_ground_.find(answer);
                        if (action_map_it!=SA_abstract_to_ground_.end())
                        {
                           	abstractSApair=action_map_it->first;
                           	action_map_it->second.push_back(currentSApair); 
                            //temp_abs_SA.push_back(currentSApair);
                            //SuperMap.erase(currentSApair);    
                         //	std::cout<<"\nAction Equivalence"<<currentSApair.first.second<<" "<<currentSApair.second;
                           // std::cout<<"\nAction Equivalence"<<abstractSApair.first.second<<" "<<abstractSApair.second;                  	
                            inverse_SA_.insert(std::pair<state_action_pair_,state_action_pair_>(currentSApair,abstractSApair));                        
                        }                       
                        else // make a new entry in action abstract map
                        {
                            //std::cout<<"\new Entry"<<currentSApair.first.second<<" "<<currentSApair.second;
                            std::vector<state_action_pair_> groundSAVector(1,currentSApair);
                            temp_abs_SA.push_back(currentSApair);
                            SA_abstract_to_ground_.insert(std::pair<state_action_pair_,std::vector<state_action_pair_>>(currentSApair,groundSAVector));
                            inverse_SA_.insert(std::pair<state_action_pair_,state_action_pair_>(currentSApair,currentSApair));                        
                        }        
                    }
               }
               //end_time=Utils:: my_read_time_in_milli_seconds();
               //std::cout<<"Action Mapping Time"<<end_time-start_time<<"\n";
               //sort(temp_abs_SA.begin(),temp_abs_SA.end())
                //building state mapping based on the above action map

                bool isNewNode=false;
                //buildStateMap(currentState);
                abstractState=get_equivalent_abstractState(currentState,currentDepth);
                tree_node_ absRepNode(std::make_pair(currentDepth,abstractState));
                typename std::map<tree_node_,std::vector<tree_node_>>::iterator stateIt;
                stateIt=abstract_to_ground_.find(absRepNode);
                if(stateIt!=abstract_to_ground_.end())
                {
                    (stateIt->second).push_back(absRepNode);
                }
                else
                {
                    std::vector<tree_node_> groundStateVector(1,absRepNode);                  
                    abstract_to_ground_.insert(std::make_pair(absRepNode,groundStateVector));
                }
                /*for (rgit = abstract_to_ground_.rbegin(); rgit != abstract_to_ground_.rend(); ++rgit)
                {
                    abstractDepth=(rgit->first).first;
                    abstractState=(rgit->first).second;
                    if (abstractDepth<currentDepth){
                        
                        isNewNode=true;
                        break;
                    }
                    else if (abstractDepth == currentDepth){
                        std::vector<tree_node_> groundStateVector = rgit->second;
                        tree_node_ absRepNode=groundStateVector[0];  //abstract representative node                        
                        //check abstraction
                        if (areStatesEquivalent(currentState,abstractState,currentDepth))
                            {              
                               //std::cout<<"\nStates Equivalent";               
                               (rgit->second).push_back(tree_node_(currentDepth,currentState));
                                break;
                            }
                    }                   
                }

                if (isNewNode || abstract_to_ground_.empty()|| (rgit == abstract_to_ground_.rend())){ //Line changed if abstract to ground table ended
                    std::vector<tree_node_> groundStateVector(1,rit->first);                  
                    abstract_to_ground_.insert(std::pair<tree_node_,std::vector<tree_node_>>(rit->first,groundStateVector));
                } 
                end_time=Utils:: my_read_time_in_milli_seconds();
               //std::cout<<"Total One node  Mapping Time"<<end_time-start_time<<"\n";            
                */
            }
        }
        // Printing Abstract Map
       /*for (rgit = abstract_to_ground_.rbegin(); rgit != abstract_to_ground_.rend(); ++rgit)
        {
            std::vector<tree_node_> groundStateVector = rgit->second;
            typename std::vector<tree_node_>::iterator vecit;
            //std::cout<<"\n Equivalent States are: ";
            for(vecit=(rgit->second).begin();vecit!=(rgit->second).end();++vecit);
              //  std::cout <<"Depth "<<vecit->first<< ",State "<<vecit->second;

        }*/
   
    }

// get the sum of elements of vector
    int getTotalCount(const T &s, unsigned depth, const std::vector<int>& count_array ) const{
        int sum_of_elems=0;
        int nactions = problem().number_actions(s);
        typename hash_t<T>::iterator it;
        it=table_.find(std::make_pair(depth,s));
        
        for(Problem::action_t a=0;a<nactions;++a)
        {
            if(problem().applicable(s,a))
            {
                typename std::map<state_action_pair_,state_action_pair_>::iterator invIt;
                invIt=inverse_SA_.find(std::make_pair(std::make_pair(depth,s),a));
                if(invIt!=inverse_SA_.end())
                {
                    typename std::map<state_action_pair_,std::pair<int,float>> ::iterator data_it;
                    data_it=SA_abstract_data_.find(invIt->second);
                        
                    sum_of_elems += data_it->second.first;
                }
                else
                    sum_of_elems += count_array[1+a]; 
            }
        }
        return sum_of_elems;

    }

    /* v2 addition: Updates Data Values of true nodes as per the value of the corresponding abstract node*/
    void update_data_values() const {
        typename hash_t<T>::iterator it;
        typename std::map<state_action_pair_,std::vector<state_action_pair_>>::iterator actit;
        typename std::vector<state_action_pair_>::iterator vec_act_it;

        int total_count=0;
        float avg_values=0;
        // Calculating average
        for(actit=SA_abstract_to_ground_.begin();actit!=SA_abstract_to_ground_.end();++actit)
        {
            avg_values=0;
            total_count=0;
            for(vec_act_it=actit->second.begin();vec_act_it!=actit->second.end();++vec_act_it)
            {
                it=table_.find(vec_act_it->first);
                if(actit->second.size()!=1)
                    avg_values+=it->second.counts_[1+vec_act_it->second]*it->second.values_[1+vec_act_it->second];
                else
                    avg_values+=it->second.values_[1+vec_act_it->second];
                total_count+=(it->second).counts_[1+vec_act_it->second]; 
            }    
            //std::cerr<<"total_count is "<<total_count<<"\n";  
#ifdef MY_DEBUG  
            if (total_count<=0)
            {
            	std::cerr<<"total_count is "<<total_count<<"\n";
            	exit(0);
            }
#endif
            if(actit->second.size()!=1)
                avg_values/=total_count;
            total_count=(int)(total_count/actit->second.size());
            SA_abstract_data_.insert(std::pair<state_action_pair_,std::pair<int,float>>(actit->first,std::make_pair(total_count,avg_values)));
           
            //std::cout<<"State:"<<actit->first.first.second<<" Depth:"<<actit->first.first.first<<" Action"<<actit->first.second<<" Old Value1:"<<avg_values<<"\n";
            // //std::cout<<"Total Count:"<<total_count<<" Abstraction Size:"<<actit->second.size()<<"\n";
            
        } 
    }

    void update_original_nodes()const{
        typename std::map <state_action_pair_,std::vector<state_action_pair_> >::reverse_iterator rgit;
        typename std::map <state_action_pair_,std::pair<int,float> >::iterator abit;
        typename std::vector<state_action_pair_>::iterator vecit;
        typename hash_t<T>::iterator it;

        for (rgit =SA_abstract_to_ground_ .rbegin(); rgit != SA_abstract_to_ground_.rend(); ++rgit){
            for(vecit=(rgit->second).begin();vecit!=(rgit->second).end();++vecit){

                it=table_.find(std::make_pair(vecit->first.first, vecit->first.second));
                
                abit=SA_abstract_data_.find(rgit->first);
                it->second.counts_[1+vecit->second]=abit->second.first;
                //it->second.counts_[1+vecit->second]=abit->second.first/rgit->second.size();
                it->second.values_[1+vecit->second]=abit->second.second;
            }
        }

    }

//      int update_equivalent_SA_nodes(const T &s, unsigned depth, float new_value, Problem::action_t a) const {
//         tree_node_ currentNode(std::make_pair(depth,s));
//         state_action_pair_ currentSApair(std::make_pair(currentNode,a));
        
//         typename std::map <state_action_pair_,std::vector<state_action_pair_>> :: iterator actit;
//         typename std::vector<state_action_pair_> ::iterator it;
//         typename std::map<state_action_pair_,std::pair<int,float>>::iterator data_it;
//         if(!inverse_SA_.count(currentSApair))
//             return 0;
         
//         state_action_pair_ abstractSApair=inverse_SA_[currentSApair];
//         actit=SA_abstract_to_ground_.find(abstractSApair);
//         data_it=SA_abstract_data_.find(abstractSApair);
//         // ++((data_it->second).first);
//         // float &old_value1=data_it->second.second;
//         // int n1=data_it->second.first;
//         // old_value1+=(new_value-old_value1)/n1;
// #ifdef MY_DEBUG 
//         if(actit==SA_abstract_to_ground_.end())
//         {
//             std::cerr<<"Error: Abstract state action pair not found\n";
// 			exit(0);
//         }
// #endif
//         for(it=actit->second.begin();it!=actit->second.end();++it)
//         {

            
//             // if((currentNode==it->first)&&(a==it->second))
//             // {
                   
//             //     continue;
//             //  }   
//             typename hash_t<T>::iterator equivNode= table_.find(std::make_pair((it->first).first,(it->first).second));
            
// #ifdef MY_DEBUG 
//             if(equivNode==table_.end())
//             {
//                 std::cerr<<"Error: Equivalent State entry not found in table\n";
//                 exit(0);
//             }
// #endif
//             // ++equivNode->second.counts_[0];
//             ++(equivNode->second.counts_[1+it->second]);
//             //std::cout<<"643:State:"<<equivNode->first.second<<" Depth:"<<equivNode->first.first<<" Action:"<<it->second<<" Counts:"<<equivNode->second.counts_[1+it->second]<<"\n";
//             float &old_value = equivNode->second.values_[1+it->second];
//             int n=equivNode->second.counts_[1+it->second];
// #ifdef MY_DEBUG
//             if (n==0)
//             {
//             	std::cerr<<"n is zero\n";
//             	exit(0);
//             }
// #endif
//             old_value += (new_value - old_value) / n;
//         }
        
//         return 0;
//      }

    float search_tree(const T &s, unsigned depth) const {
    
#ifdef DEBUG
        //std::cout << std::setw(2*depth) << "" << "search_tree(" << s << "):";
#endif

    	// if(problem().terminal(s))
    	// {
    	// 	policy_t<T>::goal_reached=true;
    	// 	policy_t<T>::this_trial_decisions+=depth;
    	// 	std::cout<<"here"<<depth<<"\n";
    	// }
        if( (depth == horizon_) || problem().terminal(s) ){// || (depth+this_trial_decisions>=101) ) {
           
        //if( (depth == horizon_) || problem().terminal(s) ) {
#ifdef DEBUG
            //std::cout << " end" << std::endl;
#endif
            return 0;
        }

        if( problem().dead_end(s) ) {
#ifdef DEBUG
            //std::cout << " dead-end" << std::endl;
#endif
            return problem().dead_end_value();
        }

        typename hash_t<T>::iterator it = table_.find(std::make_pair(depth, s));
        if( it == table_.end() ) {
            std::vector<float> values (1 + problem().number_actions(s), 0);
            std::vector<int> counts(1 + problem().number_actions(s), 0);
            
            table_.insert(std::make_pair(std::make_pair(depth, s), data_t(values, counts)));
            //std::cout<<"682:State:"<<s<<" Depth:"<<depth<<"Action 0:"<<counts[1]<<"\n";
            float value = evaluate(s, depth);
#ifdef DEBUG
            //std::cout << " insert in tree w/ value=" << value << std::endl;
#endif          
            return value;
        } else {

            // select action for this node and increase counts
            Problem::action_t a = select_action(s, it->second, depth, true, random_ties_);
           
           

            // sample next state
            //std::pair<const T, bool> p = problem().sample(s, a);
             std:: pair<T,bool> p;
             problem().sample_factored(s,a,p.first);
            float cost = problem().cost(s, a);

#ifdef DEBUG
            //std::cout << " count=" << it->second.counts_[0]-1
                      << " fetch " << std::setprecision(5) << it->second.values_[1+a]
                      << " a=" << a
                      << " next=" << p.first
                      << std::endl;
#endif
            
           float new_value = cost +
                problem().discount() * search_tree(p.first, 1 + depth);  
            int n;
           // float ret_value;
            if(!inverse_SA_.count(std::make_pair(std::make_pair(depth,s),a)))          
			{    
                 ++it->second.counts_[0]; 
                ++it->second.counts_[1+a];
                //std::cout<<"716: State:"<<s<<" Depth:"<<depth<<" Action"<<a<<" Count"<<it->second.counts_[1+a]<<"\n";

                float &old_value = it->second.values_[1+a];
                n = it->second.counts_[1+a];
                old_value += (new_value - old_value) / n;
                //std::cout<<"Unabstracted State:"<<s<<"\n";
                //  if(invIt!=inverse_SA_.end()) 
                // {
                // typename std::map<state_action_pair_,std::pair<int,float>> ::iterator data_it;
                // data_it=SA_abstract_data_.find(invIt->second);
                // ++data_it->second.first;
                // float &old_value1=data_it->second.second;
                // std::cout<<"aas\n";
                //  old_value1 += (new_value - old_value1) / n;
                // if((old_value1!=old_value) || (data_it->second.first!=it->second.counts_[1+a])){
                //     std::cout<<"State:"<<s<<" Depth:"<<depth<<" Action"<<a<<" Old Value1:"<<old_value1<<" Old value:"<<old_value<<" count1:"<<data_it->second.first<<" Count:"<<it->second.counts_[1+a]<<" Error here\n";
                
                // exit(0);}
           // }

                 return old_value; 
              //  ret_value=old_value;
               
                
             }
             else
             {

                typename std::map<state_action_pair_,std::pair<int,float>> ::iterator data_it;
                data_it=SA_abstract_data_.find(inverse_SA_[std::make_pair(std::make_pair(depth,s),a)]);
                ++data_it->second.first;
                float &old_value=data_it->second.second;
                n= data_it->second.first;
                        
                old_value += (new_value - old_value) / n;
                //std::cout<<"Abstracted State:"<<s<<"\n";
                return old_value; 
               // ret_value=old_value;
            }   
             
            /* SAU update data nodes of equivalent SA pairs */
           //if(!isUndersampledState(s,depth))
            //   update_equivalent_SA_nodes(s,depth,new_value,a);
            //return ret_value; 
            return 0;
        }
    }
    
    Problem::action_t select_action(const T &state,
                                    const data_t &data,
                                    int depth,
                                    bool add_bonus,
                                    bool random_ties) const {
       
    	// if(data.counts_[0]<=0)
    	// {
    	// 	std::cerr<<"Error: Total counts is invalid:"<<data.counts_[0]<<"\n";
    	// 	exit(0);
    	// }
        //Code_ Modified SAU
        //float log_ns = logf(data.counts_[0]);
        
        float log_ns=logf(getTotalCount(state, depth, data.counts_));
        
        std::vector<Problem::action_t> best_actions;
        
        float best_value = std::numeric_limits<float>::max();
        int nactions = problem().number_actions(state);
        best_actions.reserve(random_ties ? nactions : 1);
        for( Problem::action_t a = 0; a < nactions; ++a ) {     
            if( problem().applicable(state, a) ) {
                // std::cout<<"783: State:"<<state<<" Depth:"<<depth<<" Action"<<a<<"\n";
                // if this action has never been taken in this node, select it
                if( data.counts_[1+a] == 0 ) {
                    return a;
                }

                // compute score of action adding bonus (if applicable)
                 //Code_ Modified SAU
                //assert(data.counts_[0] > 0);
                assert(getTotalCount(state, depth, data.counts_)>0);
                int masterCount;
                float masterValue;
                typename std::map<state_action_pair_,state_action_pair_>::iterator invIt;
                invIt=inverse_SA_.find(std::make_pair(std::make_pair(depth,state),a));
                if(invIt==inverse_SA_.end())
                {
                    masterCount=data.counts_[1+a];
                    masterValue=data.values_[1+a];    
                    //std::cout<<"Unabstracted State:"<<state<<"\n";
                }
                else
                {
                    typename std::map<state_action_pair_,std::pair<int,float>>::iterator data_it;
                    data_it=SA_abstract_data_.find(invIt->second);
                    masterCount=data_it->second.first;
                    masterValue=data_it->second.second;
                    //std::cout<<"Abstracted State:"<<state<<"\n";
                }
                // float par = parameter_ == 0 ? -data.values_[1+a] : parameter_;
                // float bonus = add_bonus ? par * sqrtf(2 * log_ns / data.counts_[1+a]) : 0;
                // float value = data.values_[1+a] + bonus;
                //ll-change from here
                //float par = parameter_ == 0 ? -masterValue : parameter_;
                float par=-fabs(masterValue);
                float bonus = add_bonus ? par * sqrtf(2 * log_ns / masterCount) : 0;
                float value = masterValue + bonus;
                // update best action so far
                if( value <= best_value ) {
                    if( value < best_value ) {
                        best_value = value;
                        best_actions.clear();
                    }
                    if( random_ties || best_actions.empty() )
                        best_actions.push_back(a);
                }
            }         
        }
        assert(!best_actions.empty());
            
        return best_actions[Random::uniform(best_actions.size())];
    }
    // void printAbstractMap() const
    // {
    //     typename std::map<state_action_pair_,state_action_pair_>::iterator invit;
    //     for(invit=inverse_SA_.begin();invit!=inverse_SA_.end();invit++)
    //     {
    //         std::cout<<"\nabstract state-action"<<invit->second.first.second<<invit->second.first.first<<"Action"<<invit->second.second;
    //         std::cout<<"\nGround state-action"<<invit->first.first.second<<invit->first.first.first<<"Action"<<invit->first.second;
    //     }

    // }

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

