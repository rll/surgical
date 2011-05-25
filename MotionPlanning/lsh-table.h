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
    RRTNodeUtils utils;

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
            //float score = metric(((RRTNode *)key)->getData(), target_value);
            float score = metric(key, target);
            //cout << "Score: " << score << endl; 
            if (score < minScore) { 
              minScore = score;
              argMin = key; 
            }
          }
        }
      }
     return argMin; 
    }

    float metric (Key d1, Key d2) {
      return utils.distanceBetween((RRTNode *) d1, (RRTNode *) d2); 
    }
};


template<typename LSH> 
class LshMultiTable {
  public:   
    typedef typename LSH::Parameter Parameter;
    typedef typename LSH::Domain Domain; 
    typedef const RRTNode* Key;
    typedef std::vector<Key> Bin;
    vector<LshTable<LSH>*> tables; 
    int numTables;
    int dim; 
    int maxRepeat;
    // index 0 contains the hash with the most bits, param.dim
    // index 1 contains param.dim bits etc. 

    LshMultiTable(){}

    /* param.dim must be a power of 2 */ 
    void init (const Parameter &param, unsigned L) {
      numTables = 0; 
      dim = param.dim; 
      maxRepeat = param.repeat;
      int tmpRepeat = maxRepeat; 
      Parameter *p = new Parameter();
      do {
        p->dim = dim;
        p->repeat = tmpRepeat; 
        LshTable<LSH>* table = new LshTable<LSH>();
        table->init(*p, L);
        tables.push_back(table);
        tmpRepeat /= 2; 
        numTables++; 
      } while (tmpRepeat > 1); 

    }

    void insert(Key key) {
      for (int i = 0; i < numTables; i++) {
        tables[i]->insert(key);
      }
    }

    Key query(Key target) {     
      Key argMin = NULL; 
      double minScore = DBL_MAX; 
      for (int i = 0; i < numTables; i++) { 
        argMin = tables[i]->query(target);
        if (argMin != NULL) { 
          return argMin;
        }
      }

      //cout << "No match through hash! Running through all nodes..." << endl;
      Domain target_value = ((RRTNode *) target)->getData();
      for (int i = 0; i < tables[0]->tables_[0].size(); i++) {
        BOOST_FOREACH(Key key, tables[0]->tables_[0][i]) { 
          //float score = tables[0]->metric(((RRTNode *)key)->getData(),
          //    target_value);
          float score = tables[0]->metric(key, target);

          //cout << "Score: " << score << endl; 
          if (score < minScore) { 
            minScore = score;
            argMin = key; 
          }
        }
      }
      if (argMin == NULL) {
        cout << "Target is buggy" << endl;
        for (int i = 0; i < dim; i++) { 
          cout << target_value[i] << endl; 
        }
      }
 
      return argMin; 
    }

};



#endif

