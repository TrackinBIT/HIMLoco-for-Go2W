import threading
from pynput import keyboard


class CmdGenerator:
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._lock = threading.Lock()
        self.listener = keyboard.Listener(on_press=self._on_press,
                                        on_release=self._on_release)
        self.listener.start()

    def _on_press(self, key):
        with self._lock:
              step = 0.1
        try:
            k = key.char.lower()
            if k == 'w':   self.vx += step
            if k == 's':   self.vx -= step
            if k == 'a':   self.vy += step
            if k == 'd':   self.vy -= step
            if k == 'q':   self.wz += step / 5
            if k == 'e':   self.wz -= step / 5
            return
        except AttributeError:
            pass

        if key == keyboard.Key.space:
            self.vx = self.vy  = 0.0

    def get_cmd(self):
        with self._lock:
            return [self.vx, self.vy, self.wz]
        
    def _on_release(self,key):
        pass


_cmd = CmdGenerator()
get_cmd = _cmd.get_cmd