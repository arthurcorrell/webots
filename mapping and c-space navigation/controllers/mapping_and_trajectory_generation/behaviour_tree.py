# all subclasses of Node recieve unique instance attributes of Node upon creation
# attributes only shared if manually assigned to the same namespace instance of Node 

# root categories, defined by user: blackboard, root
class BlackBoard():
    def __init__(self):
        self.data = {}
        self.r = 0
        self.w = 0

    def write(self, key, value):
        self.w += 1
        if key in self.data:
            print(f'BLACKBOARD: Overwriting {key}')
        else:
            print(f'BLACKBOARD: Adding entry {key}')
        self.data[key] = value
        return value
    
    def read(self, key):
        self.r += 1
        if key in self.data:
            return self.data[key]
        print(f'BLACKBOARD: No entry for {key}')
        return None
    
class Context():
    def initialise(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


# superclasses: nodes, leaf nodes, edge
class Node():
    def __init__(self, children=None, name=None):
        self.children = children or []
        self.name = name or self.__class__.__name__
        self.status = 'INVALID'

class LeafNode():
    def __init__(self, name=None, context=None, blackboard=None):
        self.name = name or self.__class__.__name__
        self.context = context or None
        self.blackboard = blackboard or None
        self.status = 'INVALID'

class Edge():
    def __init__(self, child=None):
        self.child = child or []
        self.status = 'INVALID'


# subclass instances of parent node

# SUCCESS condition: all children return SUCCESS
# FAIL condition: any child returns FAIL
class Sequence(Node):
    def initialise(self):
        self.status = 'RUNNING'
        self.seq = 0
        self.tick = 0
    
    def update(self):
        
        child = self.children[self.seq]

        if self.tick == 0:
            child.initialise()
            print(f'SEQUENCE: {child.name} is running')

        if child.status == 'RUNNING':
            child.update()
            self.tick += 1

        else:
            if child.status == 'FAIL':
                print(f'SEQUENCE: {child.name} ({self.seq+1} of {len(self.children)}) failed')
                child.terminate('FAIL')

                self.status = 'FAIL'
                self.seq = 0
                self.tick = 0
                
            if child.status == 'SUCCESS':
                print(f'SEQUENCE: {child.name} ({self.seq+1} of {len(self.children)}) succeded')
                child.terminate('SUCCESS')

                self.seq += 1
                self.tick = 0

                if self.seq == len(self.children):
                    self.status = 'SUCCESS'
                    self.seq = 0
                    self.tick = 0

    def terminate(self, new_status):
        self.status = new_status

# SUCCESS condition: any child returns SUCCESS
# FAIL condition: all children return FAIL
class Selector(Node):
    def initialise(self):
        self.status = 'RUNNING'
        self.seq = 0
        self.tick = 0
    
    def update(self):

        child = self.children[self.seq]

        if self.tick == 0:
            child.initialise()
            print(f'SELECTOR: {child.name} is running')

        if child.status == 'RUNNING':
            child.update()
            self.tick += 1

        else:
            if child.status == 'SUCCESS':
                print(f'SELECTOR: {child.name} ({self.seq+1} of {len(self.children)}) succeded')
                child.terminate('SUCCESS')

                self.status = 'SUCCESS'
                self.seq = 0
                self.tick = 0

            if child.status == 'FAIL':
                print(f'SELECTOR: {child.name} ({self.seq+1} of {len(self.children)}) failed')
                child.terminate('FAIL')

                self.seq += 1
                self.tick = 0

                if self.seq == len(self.children):
                    self.status = 'FAIL'
                    self.seq = 0
                    self.tick = 0

    def terminate(self, new_status):
        self.status = new_status

# SUCCESS condition: any child returns SUCCESS
# FAIL condition: all children return FAIL
class ParallelSelector(Node):
    def initialise(self):
        self.status = 'RUNNING'
        self.tick = 0
        self.seq = 0
    
    def update(self):
        if self.tick == 0:
            for child in self.children:
                child.initialise()
                print(f'PARALLELSELECTOR: {child.name} is running')

        if all(child.status == 'RUNNING' for child in self.children):
            for child in self.children:
                child.update()
                self.tick += 1

        else:
            if any(child.status == 'SUCCESS' for child in self.children):

                self.status = 'SUCCESS'
                self.seq = 0
                self.tick = 0

                for child in self.children:
                    if child.status == 'SUCCESS':
                        print(f'PARALLEL SELECTOR: {child.name} (1 of {len(self.children)}) succeded')
                        child.terminate('SUCCESS')
                    else:
                        child.terminate('INVALID')
            else:
                for child in self.children:
                    if child.status == 'FAIL':
                        print(f'{child.name} failed')

                        child.terminate('FAIL')
                        self.seq += 1

                if self.seq == len(self.children):
                    print(f'PARALLEL SELECTOR: All children of {len(self.children)} failed')

                    self.status = 'FAIL'
                    self.seq = 0
                    self.tick = 0

    def terminate(self, new_status):
        self.status = new_status

# subclass instances of edge

# TODO: update these

class Repeat(Edge):
    def __init__(self, iterations=None):
        super().__init__()
        self.iterations = iterations or float('inf')
        self.counter = 0
    def update(self):
        while self.counter < self.iterations:
            self.counter += 1
            if not self.child.execute():
                return False
        self.counter = 0
        return True
    
    def terminate(self, new_status):
        self.status = new_status
