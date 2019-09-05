"""
stk.services.py

Syntactic sugar for accessing NAOqi services.
"""

__version__ = "0.1.2" # Working on smarter subscription

__copyright__ = "Copyright 2015, Aldebaran Robotics"
__author__ = 'ekroeger'
__email__ = 'ekroeger@aldebaran.com'


class ServiceCache(object):
    "A helper for accessing NAOqi services."

    def __init__(self, session=None):
        self.session = None
        self.services = {}
        if session:
            self.init(session)

    def on_service_unregistered(self, service_id, service_name):
        if service_name in self.services:
            del self.services[service_name]

    def init(self, session):
        "Sets the session object, if it wasn't passed to constructor."
        self.session = session
        def callback(service_id, service_name):
            self.on_service_unregistered(service_id, service_name)
        session.serviceUnregistered.connect(callback)

    def __getattr__(self, servicename):
        "We overload this so (instance).ALMotion returns the service, or None."
        if not servicename in self.services:
            if servicename.startswith("__"):
                # Behave like a normal python object for those
                raise AttributeError
            try:
                self.services[servicename] = self.session.service(servicename)
            except RuntimeError: # Cannot find service - ask again next time.
                return None
        return self.services[servicename]
