#include <iostream>
#include <iomanip>
#include <strings.h>

#define DISCOUNT 1
#define NUM_COMPUTERS 10
#define REBOOT_PENALTY 0.75
#define REBOOT_PROB 0.1


class state_t {
    short x_;
    short y_;
    short wind_;
    bool computer_state_[NUM_COMPUTERS];


  public:
    enum { Away = 0, Down = 1, Cross = 2, Up = 3, Into = 4 }; // tacks

  public:
    state_t(short x = 0, short y = 0, short wind = 0)
      : x_(x), y_(y), wind_(wind) {
       for(int i=0;i<NUM_COMPUTERS;i++)
        {
            computer_state_[i]=true;
        }

    }
    state_t(const state_t &s)
      : x_(s.x_), y_(s.y_), wind_(s.wind_) {
            for(int i=0;i<NUM_COMPUTERS;i++)
            {
                computer_state_[i]=s.computer_state_[i];
            }
       }

    ~state_t() { }

    size_t hash() const {
        return (x_ << ((8*sizeof(short)) + 3)) | (y_ << 3) | wind_;
    }

  
   
    

    

    const state_t& operator=( const state_t &s) {
        x_ = s.x_;
        y_ = s.y_;
        wind_ = s.wind_;
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
    bool operator<(const state_t &s) const {
        return (x_ < s.x_) ||
               ((x_ == s.x_) && (y_ < s.y_)) ||
               ((x_ == s.x_) && (y_ == s.y_) && (wind_ < s.wind_));
    }
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
    int rows_;
    int cols_;
    float wind_transition_[64];
    float costs_[5];
    state_t init_;
    state_t goal_;
      bool connections_[100];
    static const bool default_connections_[];
    static const float default_wind_transition_[];
    static const float default_costs_[];

  public:
    problem_t(int rows, int cols,
              int init_x = 0, int init_y = 0,
              int goal_x = std::numeric_limits<int>::max(),
              int goal_y = std::numeric_limits<int>::max(),
              float *wind_transition = 0, float *costs = 0)
      : Problem::problem_t<state_t>(DISCOUNT),
         rows_(rows), cols_(cols), init_(init_x, init_y), goal_(goal_x, goal_y)  {

        for(int i=0;i<NUM_COMPUTERS;i++)
                init_.computer_state_[i]=true;
         bcopy(default_connections_, connections_, 100 * sizeof(bool));          
        if( (goal_x == std::numeric_limits<int>::max()) ||
            (goal_y == std::numeric_limits<int>::max()) ) {
            goal_ = state_t(rows - 1, cols - 1);
        }
        
        if( wind_transition != 0 ) {
            bcopy(wind_transition, wind_transition_, 64 * sizeof(float));
        } else {
            bcopy(default_wind_transition_, wind_transition_, 64 * sizeof(float));
        }

        if( costs != 0 ) {
            bcopy(costs, costs_, 5 * sizeof(float));
        } else {
            bcopy(default_costs_, costs_, 5 * sizeof(float));
        }
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const state_t &s) const { return NUM_COMPUTERS; }
    
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {

        return (false);
    }
    virtual bool dead_end(const state_t &s) const { return false; }
    virtual bool applicable(const state_t &s, ::Problem::action_t a) const {
        return true;
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
     virtual unsigned working_comps( const state_t &s) const{
        unsigned count=0;
        for(int i=0;i<NUM_COMPUTERS;i++)
        {
            if(s.computer_state_[i])
                count+=1;
        }    
        return count;

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
       
        // state_t next_s = s.apply(a);
        // assert(next_s.in_lake(rows_, cols_));
        // for( int nwind = 0; nwind < 8; ++nwind ) {
        //     float p = wind_transition_[s.wind_ * 8 + nwind];
        //     if( p > 0 ) {
        //         next_s.wind_ = nwind;
        //         outcomes.push_back(std::make_pair(next_s, p));
        //     }
        // }
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
    1,0,0,0,1,0,0,0,0,1,
    1,1,0,0,0,0,1,1,0,0,
    0,0,1,1,0,0,1,0,0,0,
    0,0,1,1,1,1,0,0,0,0,
    1,1,0,0,1,0,0,1,0,0,
    0,0,1,0,0,1,1,0,0,1,
    0,1,0,0,0,0,1,1,1,0,
    1,0,0,1,1,0,0,1,0,0,
    0,1,0,0,1,1,0,0,1,0,
    0,1,0,1,0,0,0,1,0,1

};
const float problem_t::default_wind_transition_[] = {
    0.4, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3,
    0.4, 0.3, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.4, 0.3, 0.3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.4, 0.3, 0.3, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.4, 0.2, 0.4, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.4, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.4,
    0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3
};

const float problem_t::default_costs_[] = {
    1, 2, 3, 4, std::numeric_limits<float>::max()
};

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


