class KeyedVertex(object):
    def __init__(self, pos):
        inf = float("inf")
        self.pos = pos
        self.g = inf                # Koenig-4  (pg. 3 for dynamic init)
        self.rhs = inf              # Koenig-4  #RHS in general = min c(s,s')+g(s') over(s in successors)
        self.key = (inf,inf)
        return


    def __cmp__(self, other):
        if(self.key[0]!=other.key[0]):
            return cmp(self.key[0],other.key[0])
        else:   #k1==k1', so compare k2s
            return cmp(self.key[1],other.key[1])

    def __lt__(self, other):
        if(self.key[0]!=other.key[0]):
            return (self.key[0] < other.key[0])
        else:   #k1==k1', so compare k2s
            return (self.key[1] < other.key[1])

    def __eq__(self,other):             #Careful! Compare and equals definitions conflict. Use cmp for priority, eq for identity
        return (self.pos == other.pos)

    def __ne__(self,other): # Not necessary for python 3
        return not(self==other)

    def __str__(self):
        return str(self.pos)

    # This returns a tuple (k1,k2)
    def CalculateKey(self, s_start, map_known,k_m):
        a = min(self.g, self.rhs)
        return ( (a+self.h(s_start, map_known)+k_m) , min(self.g,self.rhs) )   # Koenig-1

    def h(self, s_start, map_known):
        return map_known.distance(self.pos, s_start.pos)    #Might call for Manhattan distance