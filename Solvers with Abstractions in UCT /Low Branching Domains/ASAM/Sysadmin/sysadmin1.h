#include <iostream>
#include <iomanip>
#include <strings.h>

#define DISCOUNT .95
#define NUM_COMPUTERS 10
#define REBOOT_PENALTY 0.75
#define REBOOT_PROB 0.1

class state_t {
    bool computer_state_[NUM_COMPUTERS];

  public:
    enum { Away = 0, Down = 1, Cross = 2, Up = 3, Into = 4 }; // tacks

  public:

    state_t( )
    {
        for(int i=0;i<NUM_COMPUTERS;i++)
        {
            computer_state_[i]=true;
        }
    }
    state_t(const state_t &s)
       {

            for(int i=0;i<NUM_COMPUTERS;i++)
            {
                computer_state_[i]=s.computer_state_[i];
            }
       }
    ~state_t() { }

    size_t hash() const {
        return 1;
    //     return (x_ << ((8*sizeof(short)) + 3)) | (y_ << 3) | wind_;
     }

    // int tack(Problem::action_t a) const {
    //     int d = Utils::abs(a - wind_);
    //     return d < 8 - d ? d : 8 - d;
    // }
    // bool in_lake(short rows, short cols) const {
    //     return (x_ >= 0) && (x_ < rows) && (y_ >= 0) && (y_ < cols);
    // }
    // std::pair<int, int> direction(Problem::action_t a) const {
    //     switch( a ) {
    //         case 0: return std::make_pair(0, 1);
    //         case 1: return std::make_pair(1, 1);
    //         case 2: return std::make_pair(1, 0);
    //         case 3: return std::make_pair(1, -1);
    //         case 4: return std::make_pair(0, -1);
    //         case 5: return std::make_pair(-1, -1);
    //         case 6: return std::make_pair(-1, 0);
    //         case 7: return std::make_pair(-1, 1);
    //         default: return std::make_pair(-1, -1);
    //     }
    // }

    // state_t apply(Problem::action_t a) const {
    //     std::pair<int, int> dir = direction(a);
    //     return state_t(x_ + dir.first, y_ + dir.second);
    // }

    const state_t& operator=( const state_t &s) {
        for(int i=0;i<NUM_COMPUTERS;i++)
            {
                computer_state_[i]=s.computer_state_[i];
            }
            return *this;
    }
    bool operator==(const state_t &s) const {
        for(int i=0;i<NUM_COMPUTERS;i++)
            {
                if(computer_state_[i]!=s.computer_state_[i])
                    return false;
            }
            return true;
    }
    bool operator!=(const state_t &s) const {
        bool flag=false;
       for(int i=0;i<NUM_COMPUTERS;i++)
            {
                if(computer_state_[i]!=s.computer_state_[i])
                {
                    flag=true;
                    break;
                }    
            }
            return flag;
    }
    // bool operator<(const state_t &s) const {
    //     return (x_ < s.x_) ||
    //            ((x_ == s.x_) && (y_ < s.y_)) ||
    //            ((x_ == s.x_) && (y_ == s.y_) && (wind_ < s.wind_));
    // }
    void print(std::ostream &os) const {
        os << "(" ;
           for(int i=0;i<NUM_COMPUTERS;i++)
            {
                os<<computer_state_[i]<<",";
            }
            os<<")";
    }
    friend class problem_t;
};

inline std::ostream& operator<<(std::ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

class problem_t : public Problem::problem_t<state_t> {
    
    bool connections_[100];
    //float costs_[5];
    state_t init_;
   
    static const bool default_connections_[];
    //static const float default_costs_[];

  public:
    problem_t( int value=0)
    : Problem::problem_t<state_t>(DISCOUNT)
     {
            for(int i=0;i<NUM_COMPUTERS;i++)
                init_.computer_state_[i]=true;

        // if( (goal_x == std::numeric_limits<int>::max()) ||
        //     (goal_y == std::numeric_limits<int>::max()) ) {
        //     goal_ = state_t(rows - 1, cols - 1);
        // }
        
      
        bcopy(default_connections_, connections_, 100 * sizeof(bool));
       

        // if( costs != 0 ) {
        //     bcopy(costs, costs_, 5 * sizeof(float));
        // } else {
        //     bcopy(default_costs_, costs_, 5 * sizeof(float));
        // }
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const state_t &s) const { return NUM_COMPUTERS+1; }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return false;
    }
    virtual bool dead_end(const state_t &s) const { return false; }
    virtual bool applicable(const state_t &s, ::Problem::action_t a) const {
        return true;;
    }
    
    virtual unsigned working_comps( const state_t &s) const{
        unsigned count=0;
        for(int i=0;i<NUM_COMPUTERS;i++)
        {
            if(s.computer_state_[i])
                count+=1;
        }    
        return count;

    }


    virtual float cost(const state_t &s, Problem::action_t a) const {
        float reward=0;
        std::vector<std::pair<state_t,float> > outcomes;
        next(s,a,outcomes);
        reward=(a==0)?0:-REBOOT_PENALTY;
        for(unsigned i=0;i<outcomes.size();i++)
        {
            reward+=outcomes[i].second*working_comps(outcomes[i].first);
        }
          
        return terminal(s) ? 0 : -1*reward;
    }

    virtual float calculate_transition(const state_t s1, const state_t s2,Problem::action_t a) const{
        float p=1;
        for(int i=0;i<NUM_COMPUTERS;i++)
        {
            float m=1;
            if(a!=i+1)
            {
                if(s1.computer_state_[i]==1)
                {
                    int nbrs=0;
                    int active_nbrs=0;
                    for(int j=0;j<NUM_COMPUTERS;j++)
                    {
                        if(i!=j)
                        {
                            if(connections_[i*10+j]==true)
                            {
                                nbrs+=1;
                                if(s1.computer_state_[j]==1)
                                    active_nbrs+=1;
                             }
                        }     
                    }
                    m=0.45+0.5*(1.0+(float)active_nbrs/nbrs);
                }
                else
                    m=REBOOT_PROB;
            }   
            if(s2.computer_state_[i]==1)
                p*=m;
            else
                p*=(1-m);
        }
        return p;

    }


    virtual void next(const state_t &s, Problem::action_t a, std::vector<std::pair<state_t,float> > &outcomes) const {
        ++expansions_;
        outcomes.clear();
        outcomes.reserve(2^NUM_COMPUTERS);
        state_t next_s;
        float p;
        for(int i=0;i<(2^NUM_COMPUTERS);i++)
        {
            for(int j=0;j<NUM_COMPUTERS;j++)
            {
                next_s.computer_state_[j]=((1<<j)&i);
            }
            p=calculate_transition(s,next_s,a);
            if(p>0)
            {
                outcomes.push_back(std::make_pair(next_s, p));
            }
        }
      
    }
    virtual void print(std::ostream &os) const { }
};

const bool problem_t::default_connections_[] = {
    1,0,0,0,0,0,0,0,1,0,
    0,1,0,0,0,0,0,1,0,1,
    0,0,1,1,0,0,0,0,1,0,
    0,0,0,1,1,0,0,0,0,0,
    0,0,0,0,1,0,1,0,0,0,
    0,0,0,1,0,1,0,1,0,0,
    0,0,0,0,0,0,1,0,1,0,
    0,0,0,0,0,1,0,1,0,1,
    0,0,0,0,0,1,0,0,1,0,
    0,1,0,0,0,0,0,0,0,1

};

// const float problem_t::default_costs_[] = {
//     1, 2, 3, 4, std::numeric_limits<float>::max()
// };

inline std::ostream& operator<<(std::ostream &os, const problem_t &p) {
    p.print(os);
    return os;
}

class scaled_heuristic_t : public Heuristic::heuristic_t<state_t> {
    const Heuristic::heuristic_t<state_t> *h_;
    float multiplier_;
  public:
    scaled_heuristic_t(const Heuristic::heuristic_t<state_t> *h, float multiplier = 1.0)
      : h_(h), multiplier_(multiplier) { }
    virtual ~scaled_heuristic_t() { }
    virtual float value(const state_t &s) const {
        return h_->value(s) * multiplier_;
    }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};

class zero_heuristic_t: public Heuristic::heuristic_t<state_t> {
  public:
    zero_heuristic_t() { }
    virtual ~zero_heuristic_t() { }
    virtual float value(const state_t &s) const { return 0; }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};


