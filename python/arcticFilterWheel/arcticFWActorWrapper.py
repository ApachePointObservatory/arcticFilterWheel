from __future__ import division, absolute_import

from twistedActor import ActorWrapper
from .arcticFWActor import ArcticFWActor

__all__ = ["ArcticFWActorWrapper"]

class ArcticFWActorWrapper(ActorWrapper):
    """!A wrapper for the arctic filter wheel actor
    """
    def __init__(self, name, userPort=0):
        self.name = name
        self.actor = None # the ArcticFWActor, once it's built
        ActorWrapper.__init__(self,
            deviceWrapperList = [],
            name = name,
            userPort = userPort,
            )

    def _makeActor(self):
        self.actor = ArcticFWActor(
            name = self.name,
            userPort = self._userPort,
            fakeFilterWheel = True,
        )