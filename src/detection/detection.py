# import computational entities (functions)
from detection import gapdetection
# from detection import lineclustering


class Detection:

    # initialize class object for the first time
    def __init__(self):
        self.status = 1
        self.line = 0

        # concentrationcheck (inputs/outputs)

        # lineclustering (inputs/outputs)

    # monitor
    # def monitor(self):
        # return self.status

    # configurator
    # def configurator(self, something):
        # self.linecl.setsomething(something)

    # setters
    # def setsomethinglincl(self, something):
        # self.linecl.setsomething(something)

    # getters
    # def getline(self):
        # return self.line

    # coordinator/composer
    def detection(self):
        # self.linecl.lineclustering()
        # self.line = self.linecl.getline()

        # get camera images

        # run computational entities and retrieve outputs
        gapdetection.gapdetection()


