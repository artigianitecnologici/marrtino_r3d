  
# coding=utf-8
#from robot_cmd_ros import *
#from getpose import *

import sys
import math

#MINHEAP
class Entry:
    def __init__(self,key,value,position):
        self.key=key
        self.value=value
        self.position=position
class MinHeap:
    def __init__(self):
        self.heap=[]
        self.elem=0
    def addEntry(self,key,value):
        newe=Entry(key,value,self.elem)
        self.heap.append(newe)
        self.upHeap(newe)
        self.elem+=1
        return newe
    def removeMin(self):
        self.elem -= 1
        self.swap(self.heap[0], self.heap[self.elem])
        toRet = self.heap.pop(self.elem)
        if (self.elem > 0):
            self.downHeap(self.heap[0])
        return toRet
    def downHeap(self,entry):
        pos = entry.position
        key = entry.key
        child = pos*2+1
        if(child>self.elem-1):
            return
        sxchild = self.heap[child]
        if (child+1>self.elem-1): #dxchild non c'è
            if(key>sxchild.key):
                self.swap(entry,sxchild)
                self.downHeap(self.heap[child])
        else:
            dxchild = self.heap[child+1]
            if(dxchild.key<sxchild.key):
                if(dxchild.key<key):
                    self.swap(dxchild,entry)
                    self.downHeap(self.heap[child+1])
            elif (sxchild.key<key):
                self.swap(sxchild, entry)
                self.downHeap(self.heap[child])

    def upHeap(self,entry):
        #assert isinstance(entry,Entry)
        if(entry.position==0):
            return
        parentpos = (entry.position - 1)//2
        parent = self.heap[parentpos]
        if(parent.key>entry.key):
            self.swap(parent,entry)
            self.upHeap(self.heap[parentpos])

    def swap(self,entry1,entry2):
        pos1=entry1.position
        pos2=entry2.position
        temp=self.heap[pos1]
        self.heap[pos1].position = pos2 #swap pos
        self.heap[pos2].position = pos1
        self.heap[pos1]=self.heap[pos2]
        self.heap[pos2]=temp
    def replace(self,newKey,entry):
        entry.key=float(newKey)
        parentpos = (entry.position-1)//2
        parent = self.heap[parentpos]
        if(entry.key<parent.key):
            self.upHeap(entry)
        else:
            self.downHeap(entry)


    def __str__(self):
        r=""
        for e in self.heap:
            r+=" "+"["+str(e.key)+" "+str(e.value)+"]"
        return r
#GRAPH
class Node:
    UNEXPLORED = "UNEXPLORED"
    EXPLORED = "EXPLORED"
    EXPLORING = "EXPLORING"

    def __init__(self,name,x,y,z):
        self.name=name
        self.x = x
        self.y = y
        self.z = z  #orientamento in gradi
        self.tag=Node.UNEXPLORED
        self.link=None #this is used to remember in the "navigate" function the link!
        self.edges = []
    def __str__(self):
        return "{} : {}".format(self.name, self.point())
    def point(self):
        return "{} {} {}".format(self.x,self.y,self.z)
    def distance(self,node):
        xdist= float(self.x)-float(node.x)
        ydist= float(self.y)-float(node.y)
        return math.sqrt(xdist*xdist + ydist*ydist)




class Edge:
    def __init__(self,node1,node2,door):
        self.node1 = node1 #nodo da cui esce l'arco, si immagina sia questo quello uscite che da il verso
        self.node2 = node2
        self.weight = node1.distance(node2)
        self.door = door

    def getOpposite(self,node):
        if self.node1 == node:
            return self.node2
        else:
            return self.node1

    def __str__(self):
        return '[{}, {}]'.format(self.node1.name,self.node2.name)

class Graph:
    def __init__(self):
        self.nodes = []
    def addNode(self,node):
        if node not in self.nodes:  #isinstance(node, Node)
            self.nodes.append(node)
    def __str__(self):
        ris = ""
        for n in self.nodes:
            ris+=str(n.name)+" : "
            for e in n.edges:
                if e.node1!=n:
                    ris+=e.node1.name+" "
                else:
                    ris+=e.node2.name+" "
            ris+="\n"
        return ris
    def addEdge(self,node1,node2,door):
        for e in node1.edges:
            if ((e.node1==node2 and e.node2==node1) or (e.node1==node1 and e.node2==node2)):
                return
        nedge = Edge(node1,node2,door)
        node1.edges.append(nedge)
        node2.edges.append(nedge)
    def getNode(self,node):
        for n in self.nodes:
            if(n.name==node):
                return n
        print("Non esiste il nodo: "+str(node))
        exit(1)
    def reset(self):
        for node in self.nodes:
            node.link=None
            node.tag=Node.UNEXPLORED

    #MAIN METHOD
    def navigate(self,nfrom,nto):
        map = {} #to map entry with nodes
        heap = MinHeap()

        for node in self.nodes:
            if node == nfrom:
                nentry=heap.addEntry(0,node)
            else:
                nentry=heap.addEntry(float('inf'), node)
            map[node]=nentry

        while(heap.elem>0):
            min = heap.removeMin()
            minn = min.value
            mink = min.key
            minn.tag = Node.EXPLORED
            if minn == nto: #mi fermo prima se trovo la meta
                break
            for edge in minn.edges:
                node = edge.getOpposite(minn)
                if(node.tag==Node.EXPLORED):
                    continue
                entryn = map.get(node)
                if(entryn.key>mink+edge.weight):
                    heap.replace(mink+edge.weight,entryn)
                    node.link=edge
        path = []
        p = nto
        while p.link!=None:
            path.append(p)
            p=p.link.getOpposite(p)
        path.append(nfrom) # orra aggiungo non lo aggiungo ci siamo già.
        path.reverse()
        print("Nodi connessi:") #stampa risultato
        for elem in path:
            print(elem)
        return path #lista gestita da un altro modulo.
    #da rimuovere?
    def makeNav(self,nfrom,nto):
        with open("DRoutput.nav",'w') as output:
            ris=""
            list = []
            oppo = 0
            dest=nto

            while True:
                list.append(nto)
                link = nto.link
                if link == None: #siamo nel nodo di partenza
                    break
                nopp = link.getOpposite(nto)

                if(nto == link.node1 and nto!=dest): #per stabilire il verso quando arriveremo in un punto
                    oppo=180
                    #print "per il nodo"+str(link)+ str(oppo)
                else:
                    oppo=0
                    #print "per il nodo" + str(link) + str(oppo)
                z = float(nto.z) + oppo

                d = nto.distance(nopp)
                if link.door == 't': #se porta faccio forward 
                    ris = 'forward({})\n'.format(d) + ris    
                    if(oppo!=0):
                        ris = 'turn(180)\n' + ris 
                else: #altrimenti moveTo
                    ris = 'moveTo({},{},{})\n'.format(nto.x,nto.y,z) + ris
                nto = nopp
            list.reverse()
            print("Nodi connessi:")
            for elem in list:
                print(elem)
            print("\nFile nav di output:")
            print(ris)
            output.write(ris)

    @classmethod
    def read(cls,file):
        gr = Graph()
        with open(file,'r') as file1:
            line = file1.readline()
            while True:
                line=file1.readline()
                if 'Archi' in line:
                    break
                s=line.strip().split()
                nnode = Node(s[0],s[1],s[2],s[3])
                gr.addNode(nnode)
            line = file1.readline()
            while(len(line)>0):
                s = line.strip().split()
                node1=gr.getNode(s[0])
                node2=gr.getNode(s[1])
                gr.addEdge(node1,node2,'f')
                line = file1.readline()
        return gr



#######TEST########
#argv[1] v/f se sto percorrendo o meno il percorso in verso opposto a quello con cui ho creato il grafo!
#è possibile aggiungere anche fino a dove...!
#argv[2] partenza
#argv[3] arrivo
if __name__ == "__main__":
    if(len(sys.argv)!=3):
        print("Lanciare con parametri 'partenza arrivo'")
        exit(1)
    g = Graph.read('DRsetup.txt') #da modificare con sys.argv[1]
    #print(g)
    TEST1 = sys.argv[1]
    TEST2 = sys.argv[2]

    k = g.navigate(g.getNode(TEST1),g.getNode(TEST2))
    #g.makeNav(g.getNode(g.getNode(TEST1),g.getNode(TEST2),k)


    #print(test.__dict__) stampa tutti i campi
