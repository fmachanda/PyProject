import asyncio
from functools import wraps

def async_loop_decorator(close=True):
    """Provide decorator to gracefully loop coroutines"""
    def decorator(func):
        @wraps(func) # Preserve metadata like func.__name__
        async def wrapper(self, *args, **kwargs):
            if hasattr(self, 'stop'):
                stop = self.stop
            elif hasattr(self, 'main'):
                stop = self.main.stop
            else:
                stop = asyncio.Event()

            try:
                while not stop.is_set():
                    try:
                        await asyncio.sleep(0)
                        await func(self, *args, **kwargs)
                    except asyncio.exceptions.CancelledError:
                        stop.set()
                        raise
                    except Exception as e:
                        raise # TODO
            except KeyboardInterrupt:
                stop.set()
                raise
            finally:
                pass
        return wrapper
    return decorator
