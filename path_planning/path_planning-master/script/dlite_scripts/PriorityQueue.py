# Bobby Holden - 13 April

# This is a hack to implement a priority queue
# with extra features necessary for the D*Lite 
# algorithm

# Ideally we'd reimplement the queue from a 
# a heap and manipulate it directly

try:
    import Queue as Q  # ver. < 3.0
except ImportError:
    import queue as Q

inf = float("inf")
# from KeyedVertex import KeyedVertex # Need to have it here so this has access to comparing methods

class PriorityQueue(Q.PriorityQueue):
    def display(self):
        tempList = []
        while not self.empty():
            s = self.get()
            print(s)
            tempList.append(s)
        for s in tempList:         #refill queue
            self.put(s)

    # def removeFromPQ(self, target):
    def remove(self, target):
        tempList = []
        l = self.qsize()
        while not self.empty():
            s = self.get()
            if(s == target):
                for s in tempList: #refill queue
                    self.put(s) 
                return 1    #found & deleted
            else:
                tempList.append(s)
        for s in tempList:         #refill queue
            self.put(s) 
        return 0 

    # def topKey(self):   #popping & putting should work if key was at or tied with top priority
    def key_peek(self):   #popping & putting should work if key was at or tied with top priority
        if(self.qsize() > 0):
            s = self.get()
            k = s.key
            self.put(s)
            return k;
        else:
            return (inf,inf)