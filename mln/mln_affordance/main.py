# -*- coding: utf-8 -*-
import os
import sys
from pracmln.utils.project import PRACMLNConfig
from pracmln.utils import config, locs
from pracmln.utils.config import global_config_filename
import os
from pracmln.mlnlearn import MLNLearn
from pracmln import MLN, Database, query
from io import open
import pickle
import argparse
import getopt
#from numba import cuda

class social_modelling():

    def read_formula(self,paths,predicate):
        """
        Returns the parsed formula
        --Inputs--
        paths: .txt file containing the formulas
        predicate: predicate object with parsed predicates
        """
        predicate_list = [x.split('(')[0] for x in predicate]
        predicate_list = predicate_list + ['!'+x for x in predicate_list]
        predicate_list = [' '+x for x in predicate_list]
        predicate_list = [(x.lower(),x) for x in predicate_list]
        formula = []
        base_path = os.getcwd()
        file = open(base_path + '/' + paths,'r',encoding = 'utf8')
        formula = file.read()
        formula = formula.split('\n')
        formula = [x for x in formula if x !='']
        formula = [' ' +x.replace(' or ',' v ').replace(' and ',' ^ ').replace(':','') for x in formula]
        formula = ['0 '+x for x in formula if 'Exists ' not in x]

        return formula

    def read_predicate(self,paths):
        """
        Returns the parsed predicates
        --Inputs--
        paths: .txt file containing the predicates
        """
        predicate = []
        base_path = os.getcwd()
        file = open(base_path + '/' + paths,'r',encoding = 'utf8')
        predicate = [line for line in file]
        predicate_list = [x.split('(')[0] for x in predicate]
        predicate_list2 = [x.split('(')[1].split(')')[0].replace(' ','').lower() for x in predicate]
        predicate = []
        for i in zip(predicate_list,predicate_list2):
            predicate.append(i[0] + '(' + i[1] + ')')
        return predicate


    def model_config(self,predicate,formula,database,mln_path,db_path):
        """
        Returns the database and mln objects in MLN format
        --Inputs--
        predicate: predicate object with parsed predicates
        formula: formula object with parsed predicates
        database:.txt file containing the database(s)
        mln_path: .mln file name to save the learned weights per formula
        db_path: .db file to save the progress of the database learning
        """
        base_path = os.getcwd()
        mln = MLN(grammar='PRACGrammar',logic='FirstOrderLogic') #Parsing with PRACGrammar since we are using clusters
        for i in predicate:
            mln << i
            print('input predicate successful:'+i)
        for i in formula:
            mln << i
            print('input formula successful :'+i)

        mln.write()
        mln.tofile(base_path + '/'+ mln_path)

        db = Database.load(mln,database)
        #db.write()
        #db.tofile(base_path + '/'+ db_path)
        return (db,mln)

    def activate_model(self,database, mln):
        """
        Returns the learned mln
        --Inputs--
        database and mln objects in MLN format
        """

        DEFAULT_CONFIG = os.path.join(locs.user_data, global_config_filename)
        conf = PRACMLNConfig(DEFAULT_CONFIG)

        config = {}
        config['verbose'] = True
        config['discr_preds'] = 0
        config['db'] = database
        config['mln'] = mln
        config['ignore_zero_weight_formulas'] = 1    #0
        config['ignore_unknown_preds'] = True   #0
        config['incremental'] = 1   #0
        config['grammar'] = 'PRACGrammar' #StandardGrammar
        config['logic'] = 'FirstOrderLogic'
        config['method'] = 'BPLL'    # BPLL (BPLL, 'pseudo-log-likelihood')
        config['optimizer'] = 'bfgs'
        config['multicore'] = False
        config['profile'] = 0
        config['shuffle'] = 0
        config['prior_mean'] = 0
        config['prior_stdev'] = 10   # 5
        config['save'] = True
        config['use_initial_weights'] = 0
        config['use_prior'] = 0


        config['infoInterval'] = 500
        config['resultsInterval'] = 1000
        conf.update(config)

        print('training...')
        learn = MLNLearn(conf, mln=mln, db=database)

        result = learn.run()
        print('finished...')
        return result

    def inference(self, path,data, mln):
        """
        Returns the queries and probabilities
        --Inputs--
        path: .txt file containing the Queries
        result: the output object from the learned mln
        data: the inference data object
        mln: the mln object
        """

        query_list = []
        base_path = os.getcwd()
        file = open(base_path + '/' + path,'r',encoding = 'ascii')
        query_list = file.read()
        query_list = query_list.split('\n')
        query_list = [x for x in query_list if x !='']
        for i in query_list:
            print(query(queries=i.encode("ascii"), method='GibbsSampler', mln=mln, db=data, verbose=False, multicore=True).run().results) # check to change the method
          # #Other Methods: EnumerationAsk, MC-SAT, WCSPInference, GibbsSampler

if __name__ == '__main__':

    s=social_modelling()
    #select_device(2) # to work with GPU 2

    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--learn", help="learn with MLN", action="store_true")
    parser.add_argument("-q", "--query", help="query MLN", action="store_true")
    args = parser.parse_args()

    if args.learn:
        print('you chose learn the weights for the mln')
        predicate = s.read_predicate('predicate.txt')
        formula = s.read_formula('formula.txt',predicate)
        data,mln = s.model_config(predicate,formula,'data.txt','results.mln','results.db')
        with open('base.mln', 'wb') as base_mln_file:
             pickle.dump(mln, base_mln_file)

        output = s.activate_model(data,mln)
        output.tofile(os.getcwd() + '/' + 'learnt_mln.mln')
    elif args.query:
        print('you chose to query the mln')
        mln = MLN.load(files='learnt_mln.mln')
        infer_world = Database.load(mln,'inference_data.txt')
        s.inference('query.txt',infer_world,mln)
    else:
        print ('please input learn (-l) or query (-q) to proceed')


#query(queries='Cancer(x)', method='MC-SAT', mln=mln, db=data, verbose=False, multicore=True).run().results





# =============================================================================
#         predicate_list = [(x,x.lower()) for x in predicate_list]
#         predicate = [x.replace(' ','').lower() for x in predicate if x !='']
#         predicate2 = []
#         for i in predicate_list:
#             for j in predicate:
#                 if i[1] in j:
#                    predicate2.append(j.replace(i[1],i[0]))
#         #predicate = [x[0] + x[1:].replace(' ','').lower() for x in predicate if x !='']
#         #predicate = ['0 '+x for x in predicate if x !='']
#
# =============================================================================
