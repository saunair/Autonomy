#!/usr/bin/env python
import numpy as np
import operator
def print_list(l):
    print l

def sort_manual(shops):

    shops_sorted = []
    
    values = [ v for v in shops.values() ]
    #TODO: Here implement manual sorting using loops
    for i in range(0, len(values)):
        for j in range(0, len(values)-i-1):
            if values[j] < values[j+1]:
                values[j], values[j+1] =  values[j+1], values[j] 

    #print values
    shops_sorted = len(values)*[0]
    i = 0 
    for kk in shops.keys():
         
        shops_sorted[values.index(shops[kk])] = [kk,shops[kk]] 
        
    print 'Manual sorting result:'
    print_list(shops_sorted)

def sort_python(shops):
    
    shops_sorted = sorted(shops.items(), key=operator.itemgetter(1), reverse=True)
    #print shops_sorted
    #TODO: Here implement sorting using pythons build in sorting functions
    #shops_sorted = [ [k,v] for k, v in shops.items() ]
    print 'Python sorting result: '
    print_list(shops_sorted)

def sort_numpy(shops):
    
    shops_sorted = []

    # TODO: Here implement sorting using numpys build-in sorting function
    x = np.array(shops)
    y = np.array(shops.items())
    print 'Numpy sorting result: '
    y = y[y[:,1].argsort()[::-1]]
    shops_sorted = y.tolist()
    print_list(shops_sorted)

def main():

    shops = {}
    shops['21st Street'] = 0.9
    shops['Voluto'] = 0.6
    shops['Coffee Tree'] = 0.45
    shops['Tazza D\' Oro'] = 0.75
    shops['Espresso a Mano'] = 0.95
    shops['Crazy Mocha'] = 0.35
    shops['Commonplace'] = 0.5

    sort_manual(shops)
    sort_python(shops)
    sort_numpy(shops)
    

if __name__ == "__main__":
    main()
