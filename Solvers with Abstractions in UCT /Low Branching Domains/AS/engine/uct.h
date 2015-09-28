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
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>
#include <map>
#include <array>
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
    unsigned horizon_;
    float parameter_;
    bool random_ties_;
    mutable hash_t<T> table_;
    
    typedef std::pair<unsigned, T> tree_node_; // SAU
    mutable std::map<tree_node_, std::vector<tree_node_>> ground_to_abstract_; // SAU
    mutable std::map<tree_node_,data_t> abstract_node_data_; //SAU 
    mutable std::map<tree_node_,tree_node_> inverse_map_;
    mutable std::map <T,T> reverseMap, reverseMapLeaf;
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

        total_time=parameter_;
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
       //decrease_in_goal_distance=0;
        //adapt=false;
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
        if(this_decision_time>50)
            this_decision_time=50;
        //std::cout<<"this_decision_time"<<this_decision_time<<"\n";
        //this_decision_width=calculate_width(this_decision_time);
        //Navigation
        this_decision_width=this_decision_time*27+100;
        //sailing wind
        //this_decision_width=this_decision_time*78+10;
        //navigation
        //this_decision_width=12*this_decision_time+2;
        //Sailing_Instance-2
        //this_decision_width=*this_decision_time+5;
        //sysadmin-2
        //this_decision_width=this_decision_width*25+5;
        //Nvaigation-2
        //this_decision_width=this_decision_time*45+10;
        rem_time-=this_decision_time;
       this_decision_width=parameter_;
        //--------------------------
      ////  this_decision_width=this_decision_time*76.92+30.77;
       //this_decision_width=this_decision_time*132+4;
       //this_decision_width=this_decision_time*27+68;
       this_decision_width=this_decision_time*34+50;
        unsigned l=width_;
         double start_time,end_time;
         start_time= Utils::my_read_time_in_milli_seconds();
       // unsigned l=width_;
    for (unsigned m = 0; m < l; ++m) {
       for( unsigned i = 0; i <this_decision_width/l; ++i ) {
             search_tree(s, 0);
         }
                update_original_nodes();
                ground_to_abstract_.clear();         
                abstract_node_data_.clear(); 
                inverse_map_.clear();
                reverseMap.clear();
                reverseMapLeaf.clear();                
                if(m!=l-1)
                {
                        construct_abstract_tree(s, 0);
                        update_data_values();
                } 
        }
         

        end_time= Utils::my_read_time_in_milli_seconds();
         //std::cout<<"END TIME:"<<end_time<<"\n";
       //std::cout<<","<<end_time-start_time<<"\n ";//<<(int)this_decision_width;;

        typename hash_t<T>::iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        Problem::action_t action = select_action(s, it->second, 0, false, random_ties_);
      
        assert(problem().applicable(s, action));
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

    /*SAU Checks whether even if one of action is not sampled till now*/
    bool Check_state_undersampled(const T &s, unsigned depth) const{
        typename hash_t<T>::iterator it = table_.find(std::make_pair(depth, s));
        int no_applicable_actions=0;
        int nactions = problem().number_actions(s);
        Problem::action_t a;
        for(a = 0; a < nactions; ++a ) {
            if( problem().applicable(s, a))  //SAU
                no_applicable_actions++;
        }
        if(it->second.counts_[0]>=no_applicable_actions)
            return false;

        return true;

    }


    /* SAU */
    float construct_abstract_tree(const T &s, unsigned depth) const {

      

                  //ground to abstract mapping stored only for previous level
        T repLeafNode;
        bool flagLeaf=0;


        std::map <tree_node_, data_t> orderedMap;
        typename hash_t<T>::const_iterator it;
      
        for (it = table_.begin(); it != table_.end(); ++it)
        {
			
            orderedMap.insert(std::pair<tree_node_, data_t>(it->first,it->second));
        }
        typename std::map <tree_node_, data_t>::reverse_iterator rit;
        // for ( rit= orderedMap.rbegin(); rit != orderedMap.rend(); ++rit)
        // {
        //     std::cout<<"\n"<<rit->first.second<<rit->first.first;
        // } 
		
		unsigned oldDepth=depth;
        
        for ( rit= orderedMap.rbegin(); rit != orderedMap.rend(); ++rit)
        {

            unsigned currentDepth=(rit->first).first;
            
            T currentState=(rit->first).second;
            typename std::map <tree_node_,std::vector<tree_node_> >::reverse_iterator rgit;
            unsigned abstractDepth; T abstractState;
            if (currentDepth!=oldDepth){
                oldDepth=currentDepth;
                reverseMap=reverseMapLeaf;

                for (rgit = ground_to_abstract_.rbegin(); rgit != ground_to_abstract_.rend(); ++rgit)
                    {
                        abstractDepth=(rgit->first).first;
                        abstractState=(rgit->first).second;
                        if (abstractDepth==currentDepth+1){
                            std::vector<tree_node_> nextDepthVector=rgit->second;
                            for (unsigned j=0; j< nextDepthVector.size(); j++)
                                reverseMap.insert(std::pair<T,T>(nextDepthVector[j].second,abstractState));
                            }
                        else if (abstractDepth<currentDepth+1)
                            break;
                    }
                reverseMapLeaf.clear();
                flagLeaf=0;
            }

            if(Check_state_undersampled(currentState,currentDepth)){ //SAU_Ankit_Checking Undersampled State
                
                if (flagLeaf==0) {
                    repLeafNode=currentState;
                    flagLeaf=1;
                    //Sau_Ankit Adding undersampled nodes to abstract to groudn mapping
                    std::vector<tree_node_> groundStateVector(1,rit->first);
                    
                }
                reverseMapLeaf.insert(std::pair<T,T>(currentState,repLeafNode));
                


            }   else {  // ADD THIS LATER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
              
                bool isNewNode=false;

                for (rgit = ground_to_abstract_.rbegin(); rgit != ground_to_abstract_.rend(); ++rgit)
                {

                    abstractDepth=(rgit->first).first;
                  
                    if (abstractDepth<currentDepth){
                        
                        isNewNode=true;
                        break;
                    }
                    else if (abstractDepth == currentDepth){

                        std::vector<tree_node_> groundStateVector = rgit->second;
                        tree_node_ absRepNode=groundStateVector[0];  //abstract representative node
                       
                        //check abstraction
                        bool isEquiv=true;
                        int nactions = problem().number_actions(absRepNode.second);
						abstractState=absRepNode.second; //Added By SAU_2
                        Problem::action_t a;
                       
                        
                        for(a = 0; a < nactions; ++a ) {
                            if( problem().applicable(currentState, a) && problem().applicable(absRepNode.second,a)) { //SAU
                                // Givan  check
                                if(!isEquivalentSA( currentState, a, abstractState, a))
                                {
                                    isEquiv=false;
                                    break;
                                }
                                  
                               
                            }
                            else if (problem().applicable(currentState, a) != problem().applicable(absRepNode.second,a)){
                                isEquiv=false;
                                break;
                            }
                        }
                        
                        if (isEquiv)
                            {
                                //std::cout<<"isEquivalent--------------\n";//<<rit->first.second<<rit->first.first<<rgit->first.second<<rgit->first.first;
                                (rgit->second).push_back(tree_node_(currentDepth,currentState));
                                inverse_map_.insert(std::pair<tree_node_,tree_node_>(rit->first,rgit->first));
                                break;
                            }

                    }

                   
                }

                if (isNewNode || ground_to_abstract_.empty()|| (rgit == ground_to_abstract_.rend())){ //Line changed if abstract to ground table ended
                    //std::cout<<"\nisNotEqui"<<rit->first.second<<rit->first.first;
                    std::vector<tree_node_> groundStateVector(1,rit->first);                   
                    ground_to_abstract_.insert(std::pair<tree_node_,std::vector<tree_node_>>(rit->first,groundStateVector));
                    inverse_map_.insert(std::pair<tree_node_,tree_node_>(rit->first,rit->first));
                    
                }
               

            } // ADD THIS LATER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        }


        return 0;
    }

    bool isEquivalentSA(const T &FirstState, int FirstAction, const T &SecondState, int SecondAction) const {

        if (problem().cost(FirstState,FirstAction)!=problem().cost(SecondState,SecondAction))
            return false;


        std::vector<std::pair<T, float> > outcomesFirst, outcomesSecond;
        problem().next(FirstState, FirstAction, outcomesFirst);
        problem().next(SecondState, SecondAction, outcomesSecond);
        unsigned osizeFirst = outcomesFirst.size(), osizeSecond = outcomesSecond.size();
        std::map <T,std::array<float,2>> transComparison;
        typename std::map <T,std::array<float,2>>::iterator transIter;
        std::array<float,2> transTemp;
        transTemp[0]=0; //Sau_Ankit
        transTemp[1]=0; //Sau_Ankit
        T temp_outcome_node;

        for (unsigned o=0; o<osizeFirst; o++)
        {
           if(outcomesFirst[o].second==0) 
               continue;
            if(reverseMap.find(outcomesFirst[o].first)!=reverseMap.end())
                temp_outcome_node=reverseMap[outcomesFirst[o].first];
            else
                temp_outcome_node=outcomesFirst[o].first;
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
            if(reverseMap.find(outcomesSecond[o].first)!=reverseMap.end())
                temp_outcome_node=reverseMap[outcomesSecond[o].first];
            else
                temp_outcome_node=outcomesSecond[o].first;
            transIter=transComparison.find(temp_outcome_node);
            if (transIter==transComparison.end()){
                return false;
            } else {
                transTemp=transIter->second;
                transTemp[1]=transTemp[1]+outcomesSecond[o].second;
                transIter->second=transTemp;
            }
        }


        for (transIter=transComparison.begin(); transIter!=transComparison.end(); ++transIter)
        {
            transTemp=transIter->second;
            if (transTemp[0]!=transTemp[1]){
                return false;
            }
        }
        return true;
    }


    /* v2 addition: Updates Data Values of true nodes as per the value of the corresponding abstract node*/
    void update_data_values() const {
        typename std::map <tree_node_,std::vector<tree_node_> >::reverse_iterator rgit;
        typename std::map <tree_node_,data_t >::iterator abit;
        typename std::vector<tree_node_>::iterator vecit;
        typename hash_t<T>::iterator it;

        for (rgit = ground_to_abstract_.rbegin(); rgit != ground_to_abstract_.rend(); ++rgit){
            int nactions=problem().number_actions(rgit->first.second);
            std::vector<int> counts(1+nactions,0);
            std::vector<float> values(1+nactions,0);
            for(Problem::action_t a=0;a<nactions;a++){
               for(vecit=(rgit->second).begin();vecit!=(rgit->second).end();++vecit){
                    if(!problem().applicable(vecit->second,a))
                        continue;
                    it=table_.find(std::make_pair(vecit->first, vecit->second));
                    counts[1+a]+=it->second.counts_[1+a];
                    if(rgit->second.size()!=1)
                    	values[1+a]+=it->second.counts_[1+a]*it->second.values_[1+a];
                    else
                    	values[1+a]+=it->second.values_[1+a];

                    counts[0]+=it->second.counts_[1+a];     
               }
               if((counts[1+a]!=0)&&(rgit->second.size()!=1))
               		values[1+a]/=counts[1+a];
               // if(rgit->second.size()==1)
               // 		values[1+a]=it->second.values_[1+a];

               counts[1+a]=(int)(counts[1+a]/rgit->second.size());

            }
            counts[0]=(counts[0]/rgit->second.size());
            
            abstract_node_data_.insert(std::make_pair(rgit->first,data_t(values,counts)));


        }

        
        // for (rgit = ground_to_abstract_.rbegin(); rgit != ground_to_abstract_.rend(); ++rgit){
        //     for(vecit=(rgit->second).begin();vecit!=(rgit->second).end();++vecit){

        //         it=table_.find(std::make_pair(vecit->first, vecit->second));
                
        //         abit=abstract_node_data_.find(rgit->first);
        //         it->second=abit->second;
        //     }
        // }

    }
     // SAU_Ankit ///
     int update_equivalent_nodes(const T &s, unsigned depth, float old_value,Problem::action_t a) const {

        typename std::map <tree_node_,std::vector<tree_node_> >::reverse_iterator rgit;
        typename std::map <tree_node_,std::vector<tree_node_> >::iterator git;
        typename std::map <tree_node_,data_t >::iterator abit;
        typename std::vector<tree_node_>::iterator it;
        typename std::vector<tree_node_>::iterator jt;


        if(!inverse_map_.count((std::make_pair(depth,s))))
             return 0;
        tree_node_ abstract_node= inverse_map_[std::make_pair(depth,s)];
        git=ground_to_abstract_.find(abstract_node);
        
       
        abit=abstract_node_data_.find(git->first);
        abit->second.values_[1+a]=old_value;
       // abit->second.values_[1+a]=abit->second.values_[1+a]*abit->second.counts_[1+a]+new_value;
        ++abit->second.counts_[1+a];
        ++abit->second.counts_[0];
        // float &old_value1 = abit->second.values_[1+a];
        // float n1=abit->second.counts_[1+a];
        // old_value1 += (new_value - old_value1) / n1;
        
        //abit->second.values_[1+a]/=(abit->second.counts_[1+a]);
        for(it=git->second.begin();it!=git->second.end();it++)
        {
            if((it->first==depth)&&(it->second==s))
            {
               
                continue;
            }
            typename hash_t<T>::iterator equiv_t = table_.find(std::make_pair(it->first, it->second));
             if(equiv_t==table_.end())
            {
                std::cout<<"\n ERRRRRRRRRRRRRRORR";
                exit(0);
            }
            ++equiv_t->second.counts_[0];
            ++equiv_t->second.counts_[1+a];
            equiv_t->second.values_[1+a]=old_value;
           // float &old_value = equiv_t->second.values_[1+a];
           // float n=equiv_t->second.counts_[1+a];
           //  old_value += (new_value - old_value) / n;
            if(equiv_t->second.values_[1+a]!=abit->second.values_[1+a])
            {
                    std::cout<<"\n as Problem here";
                    exit(0);
            }
        }    



       
        return 0;
     }

    float search_tree(const T &s, unsigned depth) const {
       
#ifdef DEBUG
        //std::cout << std::setw(2*depth) << "" << "search_tree(" << s << "):";
#endif


      /*  if(problem().terminal(s))
        {
            policy_t<T>::goal_reached=true;
        }
*/
        if( (depth == horizon_) || problem().terminal(s) ) {
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
        typename std::map<tree_node_,bool>::iterator isLeafit;
        if( it == table_.end() ) {
            std::vector<float> values (1 + problem().number_actions(s), 0);
            std::vector<int> counts(1 + problem().number_actions(s), 0);
            table_.insert(std::make_pair(std::make_pair(depth, s), data_t(values, counts)));
            float value = evaluate(s, depth);
#ifdef DEBUG
            //std::cout << " insert in tree w/ value=" << value << std::endl;
#endif
           

            return value;
        } else {

            // select action for this node and increase counts
            Problem::action_t a = select_action(s, it->second, depth, true, random_ties_);
            //std::cout<<"Type-2 Select Act:"<<a<<"\n";
            //Problem::action_t a = select_action(s, it->second, depth, true, true);//SAU Added_A
           
           

            // sample next state
            std::pair<const T, bool> p = problem().sample(s, a);
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
                float ret_value;
            if(!inverse_map_.count(std::make_pair(depth,s)))
            {          
		        ++it->second.counts_[0];
                ++it->second.counts_[1+a];
                float &old_value = it->second.values_[1+a];
                  
                int n = it->second.counts_[1+a];
               
                old_value += (new_value - old_value) / n;
                ret_value=old_value;
            }
            else
            {
                typename std::map <tree_node_,data_t >::iterator abit;
                abit=abstract_node_data_.find(inverse_map_[std::make_pair(depth,s)]);
                ++abit->second.counts_[0];
                ++abit->second.counts_[1+a];
                float &old_value1 = abit->second.values_[1+a];
                 
                int n = abit->second.counts_[1+a];
               
                old_value1 += (new_value - old_value1) / n;
                ret_value=old_value1;
                
            }
           

                
             

            // SAU_Ankit - update the corresponding equivalent nodes
            //if(!Check_state_undersampled(s,depth)) //undersampled states should not call this function
            //    update_equivalent_nodes(s,depth,old_value,a);

            return ret_value; // TO BE DISCUSSED
        }
    }
    void update_original_nodes()const{
        typename std::map <tree_node_,std::vector<tree_node_> >::reverse_iterator rgit;
        typename std::map <tree_node_,data_t >::iterator abit;
        typename std::vector<tree_node_>::iterator vecit;
        typename hash_t<T>::iterator it;

        for (rgit = ground_to_abstract_.rbegin(); rgit != ground_to_abstract_.rend(); ++rgit){
            for(vecit=(rgit->second).begin();vecit!=(rgit->second).end();++vecit){

                it=table_.find(std::make_pair(vecit->first, vecit->second));
                
                abit=abstract_node_data_.find(rgit->first);
                it->second=abit->second;
            }
        }

    }
    Problem::action_t select_action(const T &state,
                                    const data_t &data,
                                    int depth,
                                    bool add_bonus,
                                    bool random_ties) const {
        struct data_t dataMaster(data);
        if(inverse_map_.count(std::make_pair(depth,state)))
        {
            typename std::map <tree_node_,data_t >::iterator abit;
            abit=abstract_node_data_.find(inverse_map_[std::make_pair(depth,state)]);
            dataMaster=abit->second;
        }
       
         float log_ns = logf(dataMaster.counts_[0]);
         //std::cout<<state<<"\n";
         //std::cout<<"Log Ns"<<log_ns<<"\n";
        
        std::vector<Problem::action_t> best_actions;
         
        float best_value = std::numeric_limits<float>::max();
        int nactions = problem().number_actions(state);
        best_actions.reserve(random_ties ? nactions : 1);
        for( Problem::action_t a = 0; a < nactions; ++a ) {
            if( problem().applicable(state, a) ) {
                // if this action has never been taken in this node, select it
                if( dataMaster.counts_[1+a] == 0 ) {
                    return a;
                }
               
                // compute score of action adding bonus (if applicable)
                assert(dataMaster.counts_[0] > 0);
                 /*Setting Parameter explicitly 0 to accomodate l parameter -SAU */
                
                //float par = parameter_ == 0 ? -dataMaster.values_[1+a] : parameter_;
                float par =  -dataMaster.values_[1+a];
                
                float bonus = add_bonus ? par * sqrtf(2 * log_ns / dataMaster.counts_[1+a]) : 0;
                float value = dataMaster.values_[1+a] + bonus;
               // std::cout<<"in Action:"<<a<<"Value"<<std::setprecision(5)<<value<<"\n";
                // update best action so far
                if( value <= best_value ) {
                	
                    if( value < best_value ) {
                        best_value = value;
                    	          
                        best_actions.clear();
                    }
                    if( random_ties || best_actions.empty() )
                    {
                    	//std::cout<<"best2 action:"<<a<<" para:"<<par;
                        best_actions.push_back(a);
                    }
                }
            }
        }
        
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

