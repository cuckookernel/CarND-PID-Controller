
#include <iostream>
#include <vector>

using std::ostream;
using std::vector;
using std::cerr;
using std::endl;

enum class OptState {
  global_begins,
  iter_begins,
  eval_up,
  eval_down,
  done
};

ostream& operator<<( ostream& os, OptState s ) {
    switch( s ) {
      case OptState::global_begins :
        return os << "global_begins";
      case OptState::iter_begins :
        return os << "iter_begins";
      case OptState::eval_up :
        return os << "eval_up";
      case OptState::eval_down :
        return os << "eval_down";
      case OptState::done : 
        return os << "done"; 
    }
    return os << " this shouldn't happen";
}

const double NaN = std::numeric_limits<double>::quiet_NaN();
ostream& operator<<( ostream& os, const vector<double>& v ) {
    for( auto x : v ) {
      os << x << " ";
    }
    return os;
}

class TwiddleOptimizer {

  vector<double> pars;
  vector<double> best_pars;
  vector<double> dp;
  const int n_pars;
  int par_idx; // index of param currently being perturbed 
  OptState state;
  double best_err = std::numeric_limits<double>::infinity();
  double tol = 1e-6;
  
  public: 
  
  TwiddleOptimizer( const vector<double>& init_pars, const vector<double>& init_dp ) 
    : pars( init_pars ),   best_pars( init_pars ), dp( init_dp ), n_pars( init_pars.size() ), state( OptState::global_begins )  {
    
    assert( init_pars.size() == init_dp.size() );

  } 

  vector<double> BestPars() {
      return best_pars;
  }
  bool IsDone() {
      return state == OptState::done;
  }
  double BestErr() {
      return best_err;
  }
  

  inline const vector<double>& next_par( ) {
      par_idx = (par_idx + 1) % n_pars;

      if( par_idx == 0 ) {            
        state = OptState::iter_begins;     
        cerr << state << " par_idx: " << par_idx << " : " << pars[par_idx] << endl;       
        return eval_result( NaN );  //input used by eval_result
      } else {
        pars[ par_idx ] += dp[ par_idx ];
        state = OptState::eval_up;
        cerr << state << " par_idx: " << par_idx << " : " << pars[par_idx] << endl;  
        
        return pars;
      }
  }
  
  const vector<double>&  eval_result( double err ) {
    assert( isnan(err) || err > 0.0 );

    cerr << " eval_result:  state: " << state << " pars: " << pars << "  err= " << err << " dp " << dp 
        << " best_err: " << best_err <<  " best pars: " << best_pars << endl;

    double sum_dp = 0.0;

    switch( state ) {

      case OptState::global_begins:
        state = OptState::iter_begins;

        return pars;

      case OptState::iter_begins :
        if(err < best_err) {
          best_err = err;
          best_pars = pars;
        } 

        for( auto d : dp ) { sum_dp += d; }
        
        if( sum_dp < tol ) {
          state = OptState::done;
          cerr << state << " par_idx: " << par_idx << " : " << pars[par_idx] << endl;       
        
          return pars;
        }

        pars[ par_idx ] += dp[ par_idx ];
        state = OptState::eval_up;
        cerr << " eval_up par_idx: " << par_idx << " : " << pars[par_idx] << endl; 
        return pars;
    
      case OptState::eval_up :
      
        if( err < best_err ) { // twiddle up worked 
          best_err = err;
          best_pars = pars;
          dp[ par_idx ] *= 1.1;

          return next_par();
        } else { // twiddle up didn't work , now twiddle down 
          pars[ par_idx ] -= 2 * dp[ par_idx ];
          state = OptState::eval_down;
          cerr << state << " par_idx: " << par_idx << " : " << pars[par_idx] << endl; 

          return pars;
        }

      case OptState::eval_down :
        if( err < best_err ) { // twiddle down worked 
          best_err = err;
          best_pars = pars;

          return next_par();
        } else { // twiddle down didn't work
          pars[ par_idx ] += dp[ par_idx ];

          dp[ par_idx ] *= 0.9;
          cerr << state << " par_idx: " << par_idx << " : " << pars[par_idx] << "  updated dp " << dp[par_idx] << endl; 
          
          return next_par();
        }

      case OptState::done :
        cerr << " Calling eval_result in done state is NOP" << endl; 
        return pars;
    }
  } 

};