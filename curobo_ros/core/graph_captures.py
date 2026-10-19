"""
Process-global record of CUDA-graph captures.

Every cuRobo capture goes through ``GraphExecutor._initialize_cuda_graph``
(the only ``torch.cuda.CUDAGraph()`` in cuRobo), so wrapping it once gives
ground truth independent of how a solver nests its executors. Used to prime
until nothing more captures, and to report a capture inside a servo loop
(ADR 0001, invariant 3).
"""

from curobo._src.util.cuda_graph_util import GraphExecutor

_captures = []  # one name per capture, in order; list.append is atomic


def _install():
    original = GraphExecutor._initialize_cuda_graph
    if getattr(original, '_curobo_ros_counted', False):
        return

    def counted(self, inputs):
        result = original(self, inputs)
        fn = getattr(self, '_capture_fn', None)
        _captures.append(getattr(fn, '__qualname__', None) or type(fn).__name__)
        return result

    counted._curobo_ros_counted = True
    GraphExecutor._initialize_cuda_graph = counted


_install()


def mark() -> int:
    """Position to pass to ``since`` later."""
    return len(_captures)


def since(position: int) -> list:
    """Names of the graphs captured after ``mark()`` returned ``position``."""
    return _captures[position:]
