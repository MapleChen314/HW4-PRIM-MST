import numpy as np
import heapq
from typing import Union

class Graph:

    def __init__(self, adjacency_mat: Union[np.ndarray, str]):
        """
    
        Unlike the BFS assignment, this Graph class takes an adjacency matrix as input. `adjacency_mat` 
        can either be a 2D numpy array of floats or a path to a CSV file containing a 2D numpy array of floats.

        In this project, we will assume `adjacency_mat` corresponds to the adjacency matrix of an undirected graph.
    
        """
        if type(adjacency_mat) == str:
            self.adj_mat = self._load_adjacency_matrix_from_csv(adjacency_mat)
        elif type(adjacency_mat) == np.ndarray:
            self.adj_mat = adjacency_mat
        else: 
            raise TypeError('Input must be a valid path or an adjacency matrix')
        self.mst = None
        self.n_nodes=None

    def _load_adjacency_matrix_from_csv(self, path: str) -> np.ndarray:
        with open(path) as f:
            return np.loadtxt(f, delimiter=',')

    def construct_mst(self):
        """

        TODO: Given `self.adj_mat`, the adjacency matrix of a connected undirected graph, implement Prim's 
        algorithm to construct an adjacency matrix encoding the minimum spanning tree of `self.adj_mat`. 
            
        `self.adj_mat` is a 2D numpy array of floats. Note that because we assume our input graph is
        undirected, `self.adj_mat` is symmetric. Row i and column j represents the edge weight between
        vertex i and vertex j. An edge weight of zero indicates that no edge exists. 
        
        This function does not return anything. Instead, store the adjacency matrix representation
        of the minimum spanning tree of `self.adj_mat` in `self.mst`. We highly encourage the
        use of priority queues in your implementation. Refer to the heapq module, particularly the 
        `heapify`, `heappop`, and `heappush` functions.

        """
        S=set() #explored set
        self.n_nodes,a=self.adj_mat.shape
        self.mst = np.zeros((self.n_nodes,self.n_nodes)) #This is the spanning tree
        pi={}
        pred={}
        s=0
        pi[s]=0
        pred[s]=None
        S.add(s)
        for xi in range(1,self.n_nodes):
            if xi in [i for i,x in enumerate((self.adj_mat[s])) if x != 0]:
                pi[xi]=self.adj_mat[s][xi]
                pred[xi]=s
            else:
                pi[xi]=np.inf
                pred[xi]=None
        pq=[]
        for xi in range(1,self.n_nodes):
            heapq.heappush(pq,[pi[xi],xi]) #(path to S, node label)
            #heapq.heappush(pq,(xi,pi[xi]))
        #print(pq)
        # print(self.adj_mat)
        while (pq and len(S)<self.n_nodes):
            u=heapq.heappop(pq)
            #print(u)
            u_edge=u[0]
            u_name=u[1]
            if u_name in S:
                continue
            S.add(u[1])
            #print(f"Pred is {pred[u_name]} and u is {u_name}")
            self.mst[pred[u_name],u_name]=u_edge
            self.mst[u_name,pred[u_name]]=u_edge
            vs=[(y,x) for (x,y) in enumerate(self.adj_mat[u_name]) if y!=0] #edge value, node label
            for v in vs:
                if v[1] not in S:
                    if v[0] < pi[v[1]]:
                        pi[v[1]]=v[0]
                        pred[v[1]]=u_name
                        heapq.heappush(pq,[pi[v[1]],v[1]])
                        
                        
                
