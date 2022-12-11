from abc import ABC, abstractmethod
from typing import Tuple, List, Any, Generic, TypeVar, Optional


VI = TypeVar("VI")
VO = TypeVar("VO")


class BaseConverter(Generic[VI, VO], ABC):
    @abstractmethod
    def convert(self, value_src: VI) -> VO:
        pass

    @abstractmethod
    def invert(self, value_dst: VO) -> VI:
        pass


class LinearConverter(Generic[VI, VO], BaseConverter[VI, VO], ABC):
    def __init__(self, src_bounds: Tuple[VI, VI], dst_bounds: Tuple[VO, VO]) -> None:
        self.multiplier = (dst_bounds[1] - dst_bounds[0]) / (src_bounds[1] - src_bounds[0])
        self.src_min = src_bounds[0]
        self.dst_min = dst_bounds[0]

    @abstractmethod
    def convert(self, value_src: VI) -> VO:
        return self.dst_min + (value_src - self.src_min) * self.multiplier

    @abstractmethod
    def invert(self, value_dst: VO) -> VI:
        return self.src_min + (value_dst - self.dst_min) / self.multiplier


class PWMConverter(LinearConverter[int, float]):
    def __init__(self, pwm_range: Tuple[int, int], out_range: Optional[Tuple[float, float]]) -> None:
        super().__init__(pwm_range, (-1.0, 1.0) if out_range is None else out_range)

    def convert(self, value_src: int) -> float:
        return float(super().convert(value_src))

    def invert(self, value_dst: float) -> int:
        return int(super().invert(value_dst))


class PolarityPWMConverter(BaseConverter[int, float]):
    def __init__(self, pwm_range: Tuple[int, int, int]) -> None:
        self.reversed = False if pwm_range[0] < pwm_range[2] else True
        if self.reversed:
            self.lower_converter = PWMConverter((pwm_range[2], pwm_range[1]), (1.0, 0.0))
            self.upper_converter = PWMConverter((pwm_range[1], pwm_range[0]), (0.0, -1.0))
        else:
            self.lower_converter = PWMConverter((pwm_range[0], pwm_range[1]), (-1.0, 0.0))
            self.upper_converter = PWMConverter((pwm_range[1], pwm_range[2]), (0.0, 1.0))

        self.src_center = pwm_range[1]

    def convert(self, value_src: int) -> float:
        if value_src < self.src_center:
            return self.lower_converter.convert(value_src)
        else:
            return self.upper_converter.convert(value_src)

    def invert(self, value_dst: float) -> int:
        if value_dst < 0 if not self.reversed else value_dst > 0:
            return self.lower_converter.invert(value_dst)
        else:
            return self.upper_converter.invert(value_dst)


class AnalogToStateConverter(BaseConverter[int, int]):
    def __init__(self, states: Tuple[int]) -> None:
        self.states = states
        self.state_borders = [(l1 + l2) / 2 for l1, l2 in zip(self.states[:-1], self.states[1:])]

    def convert(self, value_src: int) -> int:
        i = 0
        for border in self.state_borders:
            if value_src < border:
                return i
            i += 1
        return i

    def invert(self, value_dst: int) -> int:
        return self.states[value_dst]


TypeConverter = TypeVar("TypeConverter", bound=BaseConverter)


class PWMChannelsNormalizer(Generic[TypeConverter]):
    def __init__(self, converters: List[TypeConverter]) -> None:
        self.converters = converters

    def convert(self, inputs: List[int]) -> List[Any]:
        return [converter.convert(input_value) for converter, input_value in zip(self.converters, inputs)]