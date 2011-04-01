/* File modified from original lskit
 * @author sameep
 */

#ifndef __LSHTABLE_FLAT__
#define __LSHTABLE_FLAT__

#include <lshkit/common.h>
#include <lshkit/archive.h>
#include <vector>
#include <set>
#include "rrt_utils.h"

using namespace std;
using namespace lshkit; 
/// Flat LSH index.
/** Flat LSH index is implemented as L hash tables using mutually independent
  * LSH functions.  Given a query point q, the points in the bins to which q is
  * hashed to are scanned for the nearest neighbors of q.
  *
  * @param LSH The LSH class.
  * @param KEY The key type.
  */
template <typename LSH>
class LshTable
{

public:
    typedef typename LSH::Parameter Parameter;
    typedef typename LSH::Domain Domain; 
    typedef const RRTNode* Key;
    typedef std::vector<Key> Bin;
    std::vector<LSH> lshs_;
    std::vector<std::vector<Bin> > tables_;
    int dim;  

    /// Constructor.
    LshTable() {
    }

    /// Initialize the hash tables.
    /**
      * @param param parameter of LSH function.
      * @param engine random number generator.
      * @param L number of hash table maintained.
      *
      */
    void init (const Parameter &param, unsigned L) {
        BOOST_VERIFY(lshs_.size() == 0);
        BOOST_VERIFY(tables_.size() == 0);
        lshs_.resize(L);
        tables_.resize(L);
        DefaultRng *rng = new DefaultRng();
        dim = param.dim; 
        for (unsigned i = 0; i < L; ++i) {
            lshs_[i].reset(param, *rng);
            tables_[i].resize(lshs_[i].getRange());
        }
    }
    
    /// Insert an item to the index.
    /**
      * @param key the key to the item.
      *
      * The inserted object is not explicitly given, but is obtained by
      * accessor(key).
      */
    void insert (Key key) {
        Domain value = ((RRTNode *)key)->getData();
        for (unsigned i = 0; i < lshs_.size(); ++i) {
            unsigned index = lshs_[i](value);
            tables_[i][index].push_back(key);
        }
    }

    /// Query for NN.
    /**
      * @param obj the query object.
      */
    Key query (Key target) {
      Domain target_value = ((RRTNode*) target)->getData(); 
      set<Key> *keys = new set<Key>();
      double minScore = DBL_MAX; 
      Key argMin = NULL; 
      int keysConsidered = 0; 
      for (unsigned i = 0; i < lshs_.size(); ++i) {
        unsigned index = lshs_[i](target_value);
        BOOST_FOREACH(Key key, tables_[i][index]) {
          if(keys->count(key) == 0) {
            ++keysConsidered;
            keys->insert(key);
            float score = metric(((RRTNode *)key)->getData(), target_value);
            //cout << "Score: " << score << endl; 
            if (score < minScore) { 
              minScore = score;
              argMin = key; 
            }
          }
        }
      }
      if (argMin == NULL) {
        cout << "No match through hash! Running through all nodes..." << endl;
        for (int i = 0; i < tables_[0].size(); i++) {
          BOOST_FOREACH(Key key, tables_[0][i]) { 
            ++keysConsidered;
            float score = metric(((RRTNode *)key)->getData(), target_value);
            //cout << "Score: " << score << endl; 
            if (score < minScore) { 
              minScore = score;
              argMin = key; 
            }
          }
        }
      }
     cout << "Keys considered: " << keysConsidered << endl;
     return argMin; 
    }

    float metric (Domain d1, Domain d2) { 
      float r = 0.0; 
      for (unsigned i = 0; i < dim; i++) {
        r+= (d1[i] - d2[i]) * (d1[i] - d2[i]); 
      }
      return r; 
    }
};



#endif

