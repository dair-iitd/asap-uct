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

#ifndef AOT_GH_H
#define AOT_GH_H

#include "policy.h"
#include "bdd_priority_queue.h"

#include <iostream>
#include <sstream>
#include <cassert>
#include <limits>
#include <vector>
#include <queue>

#if __clang_major__ >= 5
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif

//#define DEBUG
#define USE_BDD_PQ

namespace Online {

namespace Policy {

namespace AOT_GH {

template<typename T> class aot_t; // Forward reference

////////////////////////////////////////////////
//
// AND/OR Tree
//

template<typename T> struct node_t {
    float gvalue_;
    float hvalue_;
    float delta_;
    unsigned nsamples_;
    bool in_best_policy_;
    bool in_queue_;
    bool in_pq_;

    node_t(float gvalue = 0, float hvalue = 0, float delta = 0)
      : gvalue_(gvalue), hvalue_(hvalue), delta_(delta), nsamples_(0),
        in_best_policy_(false), in_queue_(false), in_pq_(false) { }
    virtual ~node_t() { }

    float gh_value(float w) const { return gvalue_ + w * hvalue_; }

    virtual void print(std::ostream &os, bool indent = true) const = 0;
    virtual void expand(const aot_t<T> *policy,
                        std::vector<node_t<T>*> &nodes_to_propagate) = 0;
    virtual void propagate(const aot_t<T> *policy) = 0;
};

template<typename T> struct state_node_t;
template<typename T> struct action_node_t : public node_t<T> {
    using node_t<T>::gvalue_;
    using node_t<T>::hvalue_;
    using node_t<T>::delta_;

    Problem::action_t action_;
    float action_cost_;
    state_node_t<T> *parent_;
    std::vector<std::pair<float, state_node_t<T>*> > children_;

    action_node_t(Problem::action_t action) : action_(action) { }
    virtual ~action_node_t() { }

    bool is_leaf() const { return children_.empty(); }
    void update_value(float discount) {
        gvalue_ = 0;
        hvalue_ = 0;
        for( unsigned i = 0, isz = children_.size(); i < isz; ++i ) {
            gvalue_ += children_[i].first * children_[i].second->gvalue_;
            hvalue_ += children_[i].first * children_[i].second->hvalue_;
        }
        gvalue_ = action_cost_ + discount * gvalue_;
        hvalue_ *= discount;
    }

    virtual void print(std::ostream &os, bool indent = true) const {
        if( indent ) os << std::setw(2 * parent_->depth_) << "";
        os << "[action=" << action_
           << ",g=" << gvalue_
           << ",h=" << hvalue_
           << ",delta=" << delta_
           << "]";
    }
    virtual void expand(const aot_t<T> *policy,
                        std::vector<node_t<T>*> &nodes_to_propagate) {
        policy->expand(this, nodes_to_propagate);
    }
    virtual void propagate(const aot_t<T> *policy) {
        policy->propagate(this);
    }
};

template<typename T> struct state_node_t : public node_t<T> {
    using node_t<T>::gvalue_;
    using node_t<T>::hvalue_;
    using node_t<T>::delta_;

    const T state_;
    bool is_goal_;
    bool is_dead_end_;
    unsigned depth_;
    std::vector<std::pair<int, action_node_t<T>*> > parents_;
    std::vector<action_node_t<T>*> children_;

    state_node_t(const T &state, unsigned depth = 0)
      : state_(state), is_goal_(false), is_dead_end_(false), depth_(depth) { }
    virtual ~state_node_t() { }

    Problem::action_t best_action(bool random_ties) const {
        std::vector<Problem::action_t> actions;
        actions.reserve(random_ties ? children_.size() : 1);
        float node_value = node_t<T>::gh_value(1.0);
        for( unsigned i = 0, isz = children_.size(); i < isz; ++i ) {
            action_node_t<T> *a_node = children_[i];
            float a_node_value = a_node->gh_value(1.0);
            //float abs_diff_value = fabs(node_value - a_node_value);
            //if( (abs_diff_value < 1e-3) && (random_ties || actions.empty()) );
            if( (node_value == a_node_value) && (random_ties || actions.empty()) ) {
                actions.push_back(a_node->action_);
            }
        }
        return actions.empty() ? Problem::noop : actions[Random::uniform(actions.size())];
    }

    bool is_leaf() const {
        //return is_dead_end_ || (!is_goal_ && children_.empty());
        return is_dead_end_ || is_goal_ || children_.empty();
    }
    void update_value(float w) {
        assert(!is_goal_);
        if( !is_dead_end_ ) {
            float value = std::numeric_limits<float>::max();
            for( unsigned i = 0, isz = children_.size(); i < isz; ++i ) {
                float child_value = children_[i]->gh_value(w);
                if( child_value < value ) {
                    value = child_value;
                    gvalue_ = children_[i]->gvalue_;
                    hvalue_ = children_[i]->hvalue_;
                }
            }
            assert(value < std::numeric_limits<float>::max());
        }
    }

    virtual void print(std::ostream &os, bool indent = true) const {
        if( indent ) os << std::setw(2 * depth_) << "";
        os << "[state=" << state_
           << ",depth=" << depth_
           << ",#pa=" << parents_.size()
           << ",#chld=" << children_.size()
           << ",g=" << gvalue_
           << ",h=" << hvalue_
           << ",delta=" << delta_
           << "]";
    }
    virtual void expand(const aot_t<T> *policy,
                        std::vector<node_t<T>*> &nodes_to_propagate) {
        policy->expand(this, nodes_to_propagate);
    }
    virtual void propagate(const aot_t<T> *policy) {
        policy->propagate(this);
    }
};

////////////////////////////////////////////////
//
// Hash Table
//

template<typename T> struct map_functions_t {
    bool operator()(const std::pair<const T*, unsigned> &p1,
                    const std::pair<const T*, unsigned> &p2) const {
        return (p1.second == p2.second) && (*p1.first == *p2.first);
    }
    size_t operator()(const std::pair<const T*, unsigned> &p) const {
        return p.first->hash();
    }
};

template<typename T> class hash_t :
#if __clang_major__ >= 5
  public std::unordered_map<std::pair<const T*, unsigned>,
                            state_node_t<T>*,
                            map_functions_t<T>,
                            map_functions_t<T> > {
#else
  public std::tr1::unordered_map<std::pair<const T*, unsigned>,
                                 state_node_t<T>*,
                                 map_functions_t<T>,
                                 map_functions_t<T> > {
#endif
  public:
#if __clang_major__ >= 5
    typedef typename std::unordered_map<std::pair<const T*, unsigned>,
                                        state_node_t<T>*,
                                        map_functions_t<T>,
                                        map_functions_t<T> >
            base_type;
#else
    typedef typename std::tr1::unordered_map<std::pair<const T*, unsigned>,
                                             state_node_t<T>*,
                                             map_functions_t<T>,
                                             map_functions_t<T> >
            base_type;
#endif
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
    void clear() {
        for( const_iterator it = begin(); it != end(); ++it ) {
            state_node_t<T> *s_node = it->second;
            for( int i = 0, isz = s_node->children_.size(); i < isz; ++i ) {
                delete s_node->children_[i];
            }
            delete it->second;
        }
        base_type::clear();
    }
};

////////////////////////////////////////////////
//
// Priority Queues
//

template<typename T> struct min_priority_t {
    bool operator()(const node_t<T> *n1, const node_t<T> *n2) const {
        float v1 = fabs(n1->delta_), v2 = fabs(n2->delta_);
        return v1 > v2;
    }
};

template<typename T> struct max_priority_t {
    bool operator()(const node_t<T> *n1, const node_t<T> *n2) const {
        float v1 = fabs(n1->delta_), v2 = fabs(n2->delta_);
        return v2 > v1;
    }
};

template<typename T> class priority_queue_t :
  public std::priority_queue<node_t<T>*,
                             std::vector<node_t<T>*>,
                             min_priority_t<T> > {
};

template<typename T> class bdd_priority_queue_t :
  public std::bdd_priority_queue<node_t<T>*,
                                 min_priority_t<T>,
                                 max_priority_t<T> > {
  public:
    bdd_priority_queue_t(unsigned capacity)
      : std::bdd_priority_queue<node_t<T>*,
                                min_priority_t<T>,
                                max_priority_t<T> >(capacity) { }
};

////////////////////////////////////////////////
//
// Policy
//

template<typename T> class aot_t : public improvement_t<T> {
  using policy_t<T>::problem;
  using improvement_t<T>::base_policy_;

  protected:
    float w_;
    unsigned width_;
    unsigned horizon_;
    float probability_;
    bool random_ties_;
    bool delayed_evaluation_;
    unsigned expansions_per_iteration_;
    unsigned leaf_nsamples_;
    unsigned delayed_evaluation_nsamples_;
    int leaf_selection_strategy_;
    mutable unsigned num_nodes_;
    mutable hash_t<T> table_;

    // only used for aot/heuristic
    mutable const Heuristic::heuristic_t<T> *heuristic_;

#ifdef USE_BDD_PQ
    mutable bdd_priority_queue_t<T> inside_bdd_priority_queue_;
    mutable bdd_priority_queue_t<T> outside_bdd_priority_queue_;
#else
    mutable priority_queue_t<T> inside_priority_queue_;
    mutable priority_queue_t<T> outside_priority_queue_;
#endif
    mutable float from_inside_;
    mutable float from_outside_;

    mutable node_t<T> *random_leaf_; // only used for random leaf selection strategy

    mutable unsigned total_number_expansions_;
    mutable unsigned total_evaluations_;

    // abstraction of selection strategy
    void (aot_t::*setup_expansion_loop_ptr_)(state_node_t<T>*) const;
    void (aot_t::*prepare_next_expansion_iteration_ptr_)(state_node_t<T>*) const;
    bool (aot_t::*exist_nodes_to_expand_ptr_)() const;
    node_t<T>* (aot_t::*select_node_for_expansion_ptr_)(state_node_t<T>*) const;
    void (aot_t::*clear_internal_state_ptr_)() const;

  public:
    aot_t(const policy_t<T> &base_policy,
          float w,
          unsigned width,
          unsigned horizon,
          float probability,
          bool random_ties,
          bool delayed_evaluation,
          unsigned expansions_per_iteration,
          unsigned leaf_nsamples,
          unsigned delayed_evaluation_nsamples,
          int leaf_selection_strategy)
      : improvement_t<T>(base_policy),
        w_(w),
        width_(width),
        horizon_(horizon),
        probability_(probability),
        random_ties_(random_ties),
        delayed_evaluation_(delayed_evaluation),
        expansions_per_iteration_(expansions_per_iteration),
        leaf_nsamples_(leaf_nsamples),
        delayed_evaluation_nsamples_(delayed_evaluation_nsamples),
        leaf_selection_strategy_(leaf_selection_strategy),
        num_nodes_(0),
        heuristic_(0),
#ifdef USE_BDD_PQ
        inside_bdd_priority_queue_(expansions_per_iteration),
        outside_bdd_priority_queue_(expansions_per_iteration),
#endif
        from_inside_(0),
        from_outside_(0),
        random_leaf_(0),
        total_number_expansions_(0),
        total_evaluations_(0) {
        clear_leaf_selection_strategy();
        set_leaf_selection_strategy(leaf_selection_strategy_);

        // set policy name
        std::stringstream name_stream;
        name_stream << "aot("
                    << "w=" << w_
                    << ",width=" << width_
                    << ",horizon=" << horizon_
                    << ",probability=" << probability_
                    << ",random-ties=" << (random_ties_ ? "true" : "false")
                    << ",delayed-eval=" << (delayed_evaluation_ ? "true" : "false")
                    << ",exp-per-iter=" << expansions_per_iteration_
                    << ",leaf_nsamples=" << leaf_nsamples_
                    << ",delayed_eval_nsamples=" << delayed_evaluation_nsamples_
                    << ",leaf_selection=" << leaf_selection_strategy_
                    << ")";
        policy_t<T>::set_name(name_stream.str());
    }
    virtual ~aot_t() { }

    // this is "const" because make_aot() returns a const policy*
    void set_heuristic(const Heuristic::heuristic_t<T> *heuristic) const {
        heuristic_ = heuristic;
    }

    virtual Problem::action_t operator()(const T &s) const {
        // initialize tree and setup expansion loop for selection strategy
        ++policy_t<T>::decisions_;
        clear();
        state_node_t<T> *root = fetch_node(s, 0).first;
        (this->*setup_expansion_loop_ptr_)(root);

        // expand leaves and propagate values
        unsigned expanded = 0;
        std::vector<node_t<T>*> nodes_to_propagate;
        for( unsigned i = 0; (i < width_) && (this->*exist_nodes_to_expand_ptr_)(); ) {
            unsigned expanded_in_iteration = 0;
            while( (i < width_) &&
                   (expanded_in_iteration < expansions_per_iteration_) &&
                   (this->*exist_nodes_to_expand_ptr_)() ) {
                select_and_expand(root, nodes_to_propagate);
                for( int j = 0, jsz = nodes_to_propagate.size(); j < jsz; ++j )
                    propagate(nodes_to_propagate[j]);
                nodes_to_propagate.clear();
                ++expanded_in_iteration;
                ++i;
            }
            expanded += expanded_in_iteration;
            (this->*prepare_next_expansion_iteration_ptr_)(root);
        }
        (this->*clear_internal_state_ptr_)();

        assert((width_ == 0) || ((root != 0) && problem().applicable(s, root->best_action(random_ties_))));
        assert(expanded <= width_);

        // select best action
        return width_ == 0 ? base_policy_(s) : root->best_action(random_ties_);
    }

    virtual const policy_t<T>* clone() const {
        return new aot_t(base_policy_,
                         w_,
                         width_,
                         horizon_,
                         probability_,
                         random_ties_,
                         delayed_evaluation_,
                         expansions_per_iteration_,
                         leaf_nsamples_,
                         delayed_evaluation_nsamples_,
                         leaf_selection_strategy_);
    }

    virtual void print_stats(std::ostream &os) const {
        os << "stats: policy=" << policy_t<T>::name() << std::endl;
        os << "stats: decisions=" << policy_t<T>::decisions_ << std::endl;
        os << "stats: %in=" << from_inside_ / (from_inside_ + from_outside_)
           << ", %out=" << from_outside_ / (from_inside_ + from_outside_)
           << std::endl;
        os << "stats: #expansions=" << total_number_expansions_
           << ", #evaluations=" << total_evaluations_
           << std::endl;
        base_policy_.print_stats(os);
    }

    void print_tree(std::ostream &os) const {
        std::cout << "error: not yet implemented" << std::endl;
        assert(0);
    }

    // clear data structures
    void clear_table() const {
        table_.clear();
    }
    void clear() const {
        clear_table();
        num_nodes_ = 0;
    }

    // lookup a node in hash table; if not found, create a new entry.
    std::pair<state_node_t<T>*, bool> fetch_node(const T &state,
                                                 unsigned depth) const {
        typename hash_t<T>::iterator it =
          table_.find(std::make_pair(&state, depth));
        if( it == table_.end() ) {
            ++num_nodes_;
            state_node_t<T> *node = new state_node_t<T>(state, depth);
            table_.insert(std::make_pair(std::make_pair(&node->state_, depth),
                                         node));
            if( problem().terminal(state) ) {
                node->gvalue_ = 0;
                node->hvalue_ = 0;
                node->is_goal_ = true;
            } else if( problem().dead_end(state) ) {
                node->gvalue_ = problem().dead_end_value();
                node->hvalue_ = 0;
                node->is_dead_end_ = true;
            } else {
                node->gvalue_ = 0;
                node->hvalue_ = evaluate(state, depth);
                node->nsamples_ = leaf_nsamples_;
            }
            return std::make_pair(node, false);
        } else {
            bool re_evaluated = false;
            if( it->second->is_leaf() && !it->second->is_dead_end_ && !it->second->is_goal_ && (heuristic_ == 0) ) {
                // resample: throw other rollouts to get better estimation. Only done when heuristic_ == 0
                assert(it->second->gvalue_ == 0);
                float old_val = it->second->hvalue_;
                float new_val = old_val * it->second->nsamples_ + evaluate(state, depth);
                it->second->nsamples_ += leaf_nsamples_;
                it->second->hvalue_ = new_val / it->second->nsamples_;
                re_evaluated = true;
            }
            return std::make_pair(it->second, re_evaluated);
        }
    }

    // expansion of state and action nodes. The binding of appropriate method
    // is done at run-time with virtual methods
    void select_and_expand(state_node_t<T> *root,
                           std::vector<node_t<T>*> &nodes_to_propagate) const {
        ++total_number_expansions_;
        node_t<T> *node = (this->*select_node_for_expansion_ptr_)(root);
        if( node != 0 ) node->expand(this, nodes_to_propagate);
    }
    void expand(action_node_t<T> *a_node,
                std::vector<node_t<T>*> &nodes_to_propagate,
                bool picked_from_queue = true) const {
        assert(a_node->is_leaf());
        assert(!a_node->parent_->is_goal_);
        assert(!a_node->parent_->is_dead_end_);
        a_node->gvalue_ = 0;
        a_node->hvalue_ = 0;
        std::vector<std::pair<T, float> > outcomes;
        problem().next(a_node->parent_->state_, a_node->action_, outcomes);
        a_node->children_.reserve(outcomes.size());
        for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
            const T &state = outcomes[i].first;
            float prob = outcomes[i].second;
            std::pair<state_node_t<T>*, bool> p = fetch_node(state, 1 + a_node->parent_->depth_);
            if( p.second ) {
                assert(p.first->is_leaf());
                nodes_to_propagate.push_back(p.first);
            }
            p.first->parents_.push_back(std::make_pair(i, a_node));
            a_node->children_.push_back(std::make_pair(prob, p.first));
            a_node->gvalue_ += prob * p.first->gvalue_;
            a_node->hvalue_ += prob * p.first->hvalue_;
        }
        a_node->gvalue_ = a_node->action_cost_ + problem().discount() * a_node->gvalue_;
        a_node->hvalue_ *= problem().discount();
        nodes_to_propagate.push_back(a_node);

        // re-sample sibling action nodes that are still leaves.
        if( picked_from_queue && (heuristic_ == 0)) {
            state_node_t<T> *parent = a_node->parent_;
            const T &state = parent->state_;
            unsigned depth = 1 + parent->depth_;
            for( int i = 0, isz = parent->children_.size(); i < isz; ++i ) {
                action_node_t<T> *sibling = parent->children_[i];
                if( sibling->is_leaf() ) {
                    assert(sibling->gvalue_ == sibling->action_cost_);
                    float old_val = sibling->hvalue_ / problem().discount();
                    float eval = evaluate(state, sibling->action_, depth);
                    float new_val = old_val * sibling->nsamples_ + eval;
                    sibling->nsamples_ +=
                      delayed_evaluation_nsamples_ * leaf_nsamples_;
                    sibling->hvalue_ =
                      problem().discount() * new_val/sibling->nsamples_;

#ifdef DEBUG
                    std::cout << "sibling re-sampled: "
                              << "num=" << sibling->nsamples_
                              << std::endl;
#endif
                }
            }
        }
    }
    void expand(state_node_t<T> *s_node,
                std::vector<node_t<T>*> &nodes_to_propagate) const {
        assert(s_node->is_leaf());
        assert(!s_node->is_goal_);
        assert(!s_node->is_dead_end_);
        s_node->children_.reserve(problem().number_actions(s_node->state_));
        for( Problem::action_t a = 0; a < problem().number_actions(s_node->state_); ++a ) {
            if( problem().applicable(s_node->state_, a) ) {
                // create node for this action
                ++num_nodes_;
                action_node_t<T> *a_node = new action_node_t<T>(a);
                a_node->action_cost_ = problem().cost(s_node->state_, a);
                a_node->parent_ = s_node;
                s_node->children_.push_back(a_node);

                // expand node
                if( !delayed_evaluation_ ) {
                    expand(a_node, nodes_to_propagate, false);
                } else {
                    // instead of full-width expansion to calculate value,
                    // estimate by sampling states and applying rollouts
                    // of base policy
                    float eval = evaluate(s_node->state_, a, 1+s_node->depth_);
                    a_node->gvalue_ = a_node->action_cost_;
                    a_node->hvalue_ = problem().discount() * eval;
                    a_node->nsamples_ = delayed_evaluation_nsamples_ * leaf_nsamples_;
                }
            }
        }
        nodes_to_propagate.push_back(s_node);
    }

    // propagate new values bottom-up using BFS and stopping when values changes no further
    void propagate(node_t<T> *node) const {
        node->propagate(this);
    }
    void propagate(action_node_t<T> *a_node) const {
        assert(a_node->parent_ != 0);
        propagate(a_node->parent_);
    }
    void propagate(state_node_t<T> *s_node) const {
        std::deque<state_node_t<T>*> queue;
        queue.push_back(s_node);
        s_node->in_queue_ = true;
        while( !queue.empty() ) {
            state_node_t<T> *s_node = queue.front();
            queue.pop_front();
            s_node->in_queue_ = false;
            float old_value = s_node->gh_value(w_);
            if( !s_node->is_leaf() ) s_node->update_value(w_);
            if( s_node->is_leaf() || (old_value != s_node->gh_value(w_)) ) {
                for( int i = 0, isz = s_node->parents_.size(); i < isz; ++i ) {
                    action_node_t<T> *a_node = s_node->parents_[i].second;
                    float old_value = a_node->gh_value(w_);
                    a_node->update_value(problem().discount());
                    assert(a_node->parent_ != 0);
                    if( !a_node->parent_->in_queue_ && (old_value != a_node->gh_value(w_)) ) {
                        queue.push_back(a_node->parent_);
                        a_node->parent_->in_queue_ = true;
                    }
                }
            }
        }
    }

    // evaluate a state with base policy, and evaluate an action node by
    // sampling states
    float evaluate(const T &s, unsigned depth) const {
        total_evaluations_ += leaf_nsamples_;
        if( (heuristic_ != 0) && (depth < horizon_) ) {
            return heuristic_->value(s);
        } else if( depth >= horizon_ ) {
            return 0;
        } else {
            return Evaluation::evaluation(base_policy_, s, leaf_nsamples_, horizon_ - depth);
        }
    }
    float evaluate(const T &state, Problem::action_t action, unsigned depth) const {
        // CHECK: this need to be revised when using heuristics
        float value = 0;
        for( unsigned i = 0; i < delayed_evaluation_nsamples_; ++i ) {
            std::pair<T, bool> sample = problem().sample(state, action);
            value += evaluate(sample.first, depth);
        }
        return value / delayed_evaluation_nsamples_;
    }

    // abstraction of selection strategy
    void clear_leaf_selection_strategy() {
        setup_expansion_loop_ptr_ = 0;
        prepare_next_expansion_iteration_ptr_ = 0;
        exist_nodes_to_expand_ptr_ = 0;
        select_node_for_expansion_ptr_ = 0;
        clear_internal_state_ptr_ = 0;
    }
    void set_leaf_selection_strategy(int strategy) {
        if( strategy == 1 ) {
            random_setup_selection_strategy();
        } else {
            delta_setup_selection_strategy();
        }
    }

    // selection strategy based on delta values
    void delta_setup_selection_strategy() {
        setup_expansion_loop_ptr_ = &aot_t::delta_setup_expansion_loop;
        prepare_next_expansion_iteration_ptr_ = &aot_t::delta_prepare_next_expansion_iteration;
        exist_nodes_to_expand_ptr_ = &aot_t::delta_exist_nodes_to_expand;
        select_node_for_expansion_ptr_ = &aot_t::delta_select_node_for_expansion;
        clear_internal_state_ptr_ = &aot_t::delta_clear_internal_state;
    }
    void delta_setup_expansion_loop(state_node_t<T> *root) const {
        assert(empty_priority_queues());
        insert_into_priority_queue(root);
    }
    void delta_prepare_next_expansion_iteration(state_node_t<T> *root) const {
        clear_priority_queues();
        recompute_delta(root);
    }
    bool delta_exist_nodes_to_expand() const {
        return !empty_priority_queues();
    }
    node_t<T> *delta_select_node_for_expansion(state_node_t<T>*) const {
        return select_from_priority_queue();
    }
    void delta_clear_internal_state() const {
        clear_priority_queues();
    }

    void recompute_delta(state_node_t<T> *root) const {
        assert(!root->is_goal_);
        assert(!root->is_dead_end_);

        std::deque<state_node_t<T>*> s_queue;
        bool expanding_from_s_queue = true;
        std::deque<action_node_t<T>*> a_queue;
        bool expanding_from_a_queue = false;

        root->delta_ = std::numeric_limits<float>::max();
        root->in_best_policy_ = true;
        s_queue.push_back(root);

        while( !s_queue.empty() || !a_queue.empty() ) {
            // expand from the state queue
            if( expanding_from_s_queue ) {
                while( !s_queue.empty() ) {
                    state_node_t<T> *s_node = s_queue.back();
                    s_queue.pop_back();
                    s_node->in_queue_ = false;
                    recompute_delta(s_node, a_queue);
                }
                expanding_from_s_queue = false;
                expanding_from_a_queue = true;
            }

            // expand from the action queue
            if( expanding_from_a_queue ) {
                while( !a_queue.empty() ) {
                    action_node_t<T> *a_node = a_queue.back();
                    a_queue.pop_back();
                    recompute_delta(a_node, s_queue);
                }
                expanding_from_a_queue = false;
                expanding_from_s_queue = true;
            }
        }
    }
    void recompute_delta(state_node_t<T> *s_node,
                         std::deque<action_node_t<T>*> &a_queue) const {
        assert(!s_node->is_goal_);
        assert(!s_node->is_dead_end_);
        if( s_node->is_leaf() ) {
            // insert tip node into priority queue
            if( !s_node->is_dead_end_ && !s_node->is_goal_ && (s_node->depth_ < horizon_) ) {
                insert_into_priority_queue(s_node);
            }
        } else {
            assert(!s_node->children_.empty());
            float best_value = s_node->gh_value(w_);
            if( s_node->in_best_policy_ ) {
                assert(s_node->delta_ >= 0);

                // compute Delta
                float Delta = std::numeric_limits<float>::max();
                for( int i = 0, isz = s_node->children_.size(); i < isz; ++i ) {
                    action_node_t<T> *a_node = s_node->children_[i];
                    if( a_node->gh_value(w_) != best_value ) {
                        float d = a_node->gh_value(w_) - best_value;
                        Delta = Utils::min(Delta, d);
                    }
                }

                // compute delta
                for( int i = 0, isz = s_node->children_.size(); i < isz; ++i ) {
                    action_node_t<T> *a_node = s_node->children_[i];
                    if( a_node->gh_value(w_) == best_value ) {
                        a_node->delta_ = Utils::min(s_node->delta_, Delta);
                        a_node->in_best_policy_ = true;
                        assert(a_node->delta_ >= 0);
                    } else {
                        a_node->delta_ = best_value - a_node->gh_value(w_);
                        a_node->in_best_policy_ = false;
                        assert(a_node->delta_ <= 0);
                    }
                    a_queue.push_back(a_node);
                }
            } else {
                assert(s_node->delta_ <= 0);
                for( int i = 0, isz = s_node->children_.size(); i < isz; ++i ) {
                    action_node_t<T> *a_node = s_node->children_[i];
                    a_node->delta_ = s_node->delta_ + best_value - a_node->gh_value(w_);
                    a_node->in_best_policy_ = false;
                    assert(a_node->delta_ <= 0);
                    a_queue.push_back(a_node);
                }
            }
        }
    }
    void recompute_delta(action_node_t<T> *a_node,
                         std::deque<state_node_t<T>*> &s_queue) const {
        if( a_node->is_leaf() ) {
            // insert tip node into priority queue
            if( a_node->parent_->depth_ < horizon_ ) {
                insert_into_priority_queue(a_node);
            }
        } else {
            for( int i = 0, isz = a_node->children_.size(); i < isz; ++i ) {
                state_node_t<T> *s_node = a_node->children_[i].second;
                if( !s_node->in_queue_ && !s_node->is_goal_ && !s_node->is_dead_end_ ) {
                    float delta = std::numeric_limits<float>::max();
                    bool in_best_policy = false;
                    for( int j = 0, jsz = s_node->parents_.size(); j < jsz; ++j ) {
                        int child_index = s_node->parents_[j].first;
                        action_node_t<T> *parent = s_node->parents_[j].second;
                        assert(parent->children_[child_index].second == s_node);
                        float d = parent->delta_ /
                                  (problem().discount() * parent->children_[child_index].first);
                        delta = Utils::min(delta, fabsf(d));
                        in_best_policy = in_best_policy || parent->in_best_policy_;
                    }
                    s_node->delta_ = in_best_policy ? delta : -delta;
                    s_node->in_best_policy_ = in_best_policy;
                    s_queue.push_back(s_node);
                    s_node->in_queue_ = true;
                }
            }
        }
    }

    // implementation of priority queue for storing the deltas
    unsigned size_priority_queues() const {
#ifdef USE_BDD_PQ
        return inside_bdd_priority_queue_.size() + outside_bdd_priority_queue_.size();
#else
        return inside_priority_queue_.size() + outside_priority_queue_.size();
#endif
    }
    bool empty_inside_priority_queue() const {
#ifdef USE_BDD_PQ
        return inside_bdd_priority_queue_.empty();
#else
        return inside_priority_queue_.empty();
#endif
    }
    bool empty_outside_priority_queue() const {
#ifdef USE_BDD_PQ
        return outside_bdd_priority_queue_.empty();
#else
        return outside_priority_queue_.empty();
#endif
    }
    bool empty_priority_queues() const {
        return empty_inside_priority_queue() && empty_outside_priority_queue();
    }
    void clear(priority_queue_t<T> &pq) const {
        while( !pq.empty() ) {
            node_t<T> *node = pq.top();
            pq.pop();
            assert(node->in_pq_);
            node->in_pq_ = false;
        }
    }
    void clear(bdd_priority_queue_t<T> &pq) const {
        while( !pq.empty() ) {
            node_t<T> *node = pq.top();
            pq.pop();
            assert(node->in_pq_);
            node->in_pq_ = false;
        }
    }
    void clear_priority_queues() const {
#ifdef USE_BDD_PQ
        clear(inside_bdd_priority_queue_);
        clear(outside_bdd_priority_queue_);
#else
        clear(inside_priority_queue_);
        clear(outside_priority_queue_);
#endif
    }
    void insert_into_inside_priority_queue(node_t<T> *node) const {
#ifdef USE_BDD_PQ
        std::pair<bool, bool> p = inside_bdd_priority_queue_.push(node);
        node->in_pq_ = p.first;
        if( p.second ) {
            node_t<T> *removed = inside_bdd_priority_queue_.removed_element();
            assert(removed != 0);
            assert(removed->in_pq_);
            removed->in_pq_ = false;
        }
#else
        inside_priority_queue_.push(node);
        node->in_pq_ = true;
#endif
    }
    void insert_into_outside_priority_queue(node_t<T> *node) const {
#ifdef USE_BDD_PQ
        std::pair<bool, bool> p = outside_bdd_priority_queue_.push(node);
        node->in_pq_ = p.first;
        if( p.second ) {
            node_t<T> *removed = outside_bdd_priority_queue_.removed_element();
            assert(removed != 0);
            assert(removed->in_pq_);
            removed->in_pq_ = false;
        }
#else
        outside_priority_queue_.push(node);
        node->in_pq_ = true;
#endif
    }
    void insert_into_priority_queue(node_t<T> *node) const {
        if( !node->in_pq_ ) {
            float sign = copysignf(1, node->delta_);
            if( sign == 1 ) {
#ifdef DEBUG
                std::cout << "push:in  " << node << "=";
                node->print(std::cout, false);
                std::cout << std::endl;
#endif
                insert_into_inside_priority_queue(node);
            } else {
#ifdef DEBUG
                std::cout << "push:out " << node << "=";
                node->print(std::cout, false);
                std::cout << std::endl;
#endif
                insert_into_outside_priority_queue(node);
            }
        }
    }
    node_t<T>* select_from_inside() const {
        node_t<T> *node = 0;
#ifdef USE_BDD_PQ
        node = inside_bdd_priority_queue_.top();
        inside_bdd_priority_queue_.pop();
#else
        node = inside_priority_queue_.top();
        inside_priority_queue_.pop();
#endif
        assert(node->in_pq_);
        node->in_pq_ = false;
        ++from_inside_;
        return node;
    }
    node_t<T>* select_from_outside() const {
       node_t<T> *node = 0;
#ifdef USE_BDD_PQ
        node = outside_bdd_priority_queue_.top();
        outside_bdd_priority_queue_.pop();
#else
        node = outside_priority_queue_.top();
        outside_priority_queue_.pop();
#endif
        assert(node->in_pq_);
        node->in_pq_ = false;
        ++from_outside_;
        return node;
    }
    node_t<T>* select_from_priority_queue() const {
        node_t<T> *node = 0;
        if( empty_inside_priority_queue() && empty_outside_priority_queue() ) {
            node = 0;
        } else if( empty_inside_priority_queue() ) {
            node = select_from_outside();
        } else if( empty_outside_priority_queue() ) {
            node = select_from_inside();
        } else {
            if( (probability_ == 1) || (Random::real() < probability_) )
                node = select_from_inside();
            else
                node = select_from_outside();
        }

#ifdef DEBUG
        std::cout << "pop " << (node == 0 ? "<null>" : "");
        if( node != 0 ) node->print(std::cout, false);
        std::cout << std::endl;
#endif

        return node;
    }

    // selection strategy based on random selection
    void random_setup_selection_strategy() {
        setup_expansion_loop_ptr_ = &aot_t::random_setup_expansion_loop;
        prepare_next_expansion_iteration_ptr_ = &aot_t::random_prepare_next_expansion_iteration;
        exist_nodes_to_expand_ptr_ = &aot_t::random_exist_nodes_to_expand;
        select_node_for_expansion_ptr_ = &aot_t::random_select_node_for_expansion;
        clear_internal_state_ptr_ = &aot_t::random_clear_internal_state;
    }
    void random_setup_expansion_loop(state_node_t<T> *root) const {
        random_leaf_ = root;
    }
    void random_prepare_next_expansion_iteration(state_node_t<T> *node) const {
        if( node->is_leaf() ) {
            if( !node->is_goal_ && !node->is_dead_end_ && (node->depth_ < horizon_) ) {
                if( (random_leaf_ == 0) || (Random::real() < 0.5) )
                    random_leaf_ = node;
            }
        } else {
            assert(!node->children_.empty());
            for( int i = 0, isz = node->children_.size(); i < isz; ++i ) {
                action_node_t<T> *a_node = node->children_[i];
                assert(!a_node->children_.empty());
                for( int j = 0, jsz = a_node->children_.size(); j < jsz; ++j ) {
                    state_node_t<T> *s_node = a_node->children_[j].second;
                    random_prepare_next_expansion_iteration(s_node);
                }
            }
        }
    }
    bool random_exist_nodes_to_expand() const {
        return random_leaf_ != 0;
    }
    node_t<T> *random_select_node_for_expansion(state_node_t<T> *node) const {
        node_t<T> *leaf = random_leaf_;
        random_leaf_ = 0;
        return leaf;
    }
    void random_clear_internal_state() const {
        random_leaf_ = 0;
    }

};

}; // namespace AOT_GH

template<typename T>
inline const policy_t<T>* make_aot_gh(const policy_t<T> &base_policy,
                                      float w,
                                      unsigned width,
                                      unsigned horizon,
                                      float probability,
                                      bool random_ties,
                                      bool delayed_evaluation = true,
                                      unsigned expansions_per_iteration = 100,
                                      unsigned leaf_nsamples = 1,
                                      unsigned delayed_evaluation_nsamples = 1,
                                      int leaf_selection_strategy = 0) {
    return new AOT_GH::aot_t<T>(base_policy,
                                w,
                                width,
                                horizon,
                                probability,
                                random_ties,
                                delayed_evaluation,
                                expansions_per_iteration,
                                leaf_nsamples,
                                delayed_evaluation_nsamples,
                                leaf_selection_strategy);
}

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

