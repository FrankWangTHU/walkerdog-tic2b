"""Fake robot controller for development and demos.
This simulates a 'dog' robot with simple actions.
"""
import time

class FakeDog:
    def __init__(self):
        self.status = "resting"

    def walk(self):
        self.status = "walking"
        # simulate some activity
        time.sleep(0.1)
        self.status = "resting"
        return {"action": "walked", "status": self.status}

    def sit(self):
        self.status = "sitting"
        return {"action": "sit", "status": self.status}

    def get_status(self):
        return {"status": self.status}
