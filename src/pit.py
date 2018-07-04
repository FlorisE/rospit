""" A framework for running automated tests using external instrumentation """
from abc import ABCMeta, abstractmethod
from collections import namedtuple
import future  # noqa:401, pylint: disable=W0611


class Runner(object):
    """
    Class for running test suites
    """

    def __init__(self):
        self.reports = []
        self.last_suite = None

    def run_suite(self, test_suite):
        """
        Runs the specified test_suite
        """
        self.last_suite = test_suite
        return test_suite.run()

    def run_all(self):
        """
        Autodetect test suites and runs them
        """
        pass


class TestSuite(object):
    """
    Container for test cases
    """
    __metaclass__ = ABCMeta

    def __init__(self, name=""):
        self.name = name
        self.test_cases = []

    def run(self):
        """
        Runs all the test cases in this test suite
        """
        test_case_reports = [tc.execute() for tc in self.test_cases]
        return TestSuiteReport(test_case_reports)


TestCaseReport = namedtuple(
    "TestCaseReport",
    ["test_case", "preconditions", "invariants", "postconditions"])


TestSuiteReport = namedtuple(
    "TestSuiteReport",
    ["test_case_reports"])


class TestCase(object):
    """
    Specifies a test case with preconditions, invariants and postconditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, name="", preconditions=None, invariants=None,
                 postconditions=None):
        self.name = name
        if preconditions is None:
            preconditions = []
        if invariants is None:
            invariants = []
        if postconditions is None:
            postconditions = []
        self.preconditions = preconditions
        self.invariants = invariants
        self.postconditions = postconditions

    def verify_preconditions(self):
        """
        Evaluates all preconditions
        """
        return [c.evaluate() for c in self.preconditions]

    def verify_invariants(self, time):
        """
        Evaluates the invarians at time since starting
        """
        return [c.evaluate(time) for c in self.invariants]

    def verify_postconditions(self):
        """
        Evaluates all postconditions
        """
        return [c.evaluate() for c in self.postconditions]

    def execute(self):
        """
        Verifies the preconditions, runs the test, verifies the postconditions
        """
        preconditions_evaluation = self.verify_preconditions()
        invariants = self.run()
        postconditions_evaluation = self.verify_postconditions()
        return TestCaseReport(self, preconditions_evaluation, invariants,
                              postconditions_evaluation)

    @abstractmethod
    def run(self):
        """
        Runs the test
        """
        pass


class Evaluation(object):
    """
    Evaluation produced by an Evaluator
    """
    def __init__(self, measurement, condition, nominal):
        self.measurement = measurement
        self.condition = condition
        self.nominal = nominal


class CompositeEvaluation(Evaluation):
    """
    Evaluation which consists of a list of evaluations for a single measurement
    """
    def __init__(self, measurement, condition, evaluations):
        Evaluation.__init__(
            self, measurement, condition,
            all([evaluation.nominal for evaluation in evaluations]))
        self.evaluations = evaluations


class TimestampEvaluationPair(object):
    """
    Pair of a timestamp and an evaluation
    """
    def __init__(self, timestamp, evaluation):
        self.timestamp = timestamp
        self.evaluation = evaluation


class ConditionEvaluatorPair(object):
    """
    Pair of a condition and an evaluator
    """
    def __init__(self, condition, evaluator):
        self.condition = condition
        self.evaluator = evaluator


class Condition(object):
    """
    Abstract base class for conditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name


class Evaluator(object):
    """
    Abstract base class for evaluators
    """
    __metaclass__ = ABCMeta

    def __init__(self, sensor):
        self.sensor = sensor

    @abstractmethod
    def evaluate(self, condition, measurement):
        """
        Executes the evaluator by checking the measurement using the condition
        """
        pass


class Measurement(object):
    """
    Contains a measurement
    """
    __metaclass__ = ABCMeta


class Sensor(object):
    """
    Abstract base class for sensors
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def sense(self):
        """
        Reads the sensor value
        """
        pass


class BinaryConditionEvaluator(Evaluator):
    """
    Evaluator for binary values
    """
    def __init__(self, sensor):
        Evaluator.__init__(self, sensor)

    def evaluate(self, condition, measurement=None):
        if measurement is None:
            measurement = self.sensor.sense()
        nominal = measurement.value == condition.value
        return Evaluation(measurement, condition, nominal)


class BinarySensor(Sensor):
    """
    Sensor that returns a binary value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense(self):
        pass


class BinaryMeasurement(Measurement):
    """
    Measurement of a binary value
    """
    def __init__(self, value):
        self.value = value


class BinaryCondition(Condition):
    """
    Condition that is either True or False
    """
    __metaclass__ = ABCMeta

    def __init__(self, value, name=""):
        Condition.__init__(self, name)
        self.value = value


class InCategoryConditionEvaluator(Evaluator):
    """
    Evaluator for values that should be in a certain category
    """
    def __init__(self, sensor):
        Evaluator.__init__(self, sensor)

    def evaluate(self, condition, measurement=None):
        if measurement is None:
            measurement = self.sensor.sense()
        nominal = measurement.category in condition.categories
        return Evaluation(measurement, condition, nominal)


class CategorySensor(Sensor):
    """
    Sensor that returns a categorical value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense(self):
        pass


class CategoryMeasurement(Measurement):
    """
    Measurement of a in category value
    """
    def __init__(self, category):
        self.category = category


class InCategoriesCondition(Condition):
    """
    Condition that a value is within a list of accepted values
    """
    __metaclass__ = ABCMeta

    def __init__(self, categories=None, name=""):
        Condition.__init__(self, name)
        if categories is None:
            categories = set()
        self.categories = categories


class LowerLimitEvaluator(Evaluator):
    """
    Evaluator for the lower limit of numeric conditions
    """
    def __init__(self, sensor):
        Evaluator.__init__(self, sensor)

    def evaluate(self, condition, measurement=None):
        """
        Verifies whether measurement matches the lower limit condition
        """
        if measurement is None:
            measurement = self.sensor.sense()

        if condition.lower_limit_is_inclusive:
            nominal = measurement.value >= condition.lower_limit
        else:
            nominal = measurement.value > condition.lower_limit

        return Evaluation(measurement, condition, nominal)


class UpperLimitEvaluator(Evaluator):
    """
    Evaluator for the upper limit of numeric conditions
    """
    def __init__(self, sensor):
        Evaluator.__init__(self, sensor)

    def evaluate(self, condition, measurement=None):
        """
        Verifies whether measurement matches the upper limit condition
        """
        if measurement is None:
            measurement = self.sensor.sense()

        if condition.upper_limit_is_inclusive:
            nominal = measurement.value <= condition.upper_limit
        else:
            nominal = measurement.value < condition.upper_limit

        return Evaluation(measurement, condition, nominal)


class BothLimitsEvaluator(LowerLimitEvaluator, UpperLimitEvaluator):
    """
    Evaluator for numeric conditions
    """
    def __init__(self, sensor):
        LowerLimitEvaluator.__init__(self, sensor)
        UpperLimitEvaluator.__init__(self, sensor)

    def evaluate(self, condition, measurement=None):
        """
        Verifies whether measurement matches the lower limit and
        upper limit conditions
        """
        if measurement is None:
            measurement = self.sensor.sense()

        lower_limit_eval = LowerLimitEvaluator.evaluate(
            self, condition, measurement)
        upper_limit_eval = UpperLimitEvaluator.evaluate(
            self, condition, measurement)

        return CompositeEvaluation(
            measurement, condition,
            [lower_limit_eval, upper_limit_eval])


class NumericSensor(Sensor):
    """
    Sensor that reads a numeric value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense(self):
        pass


class NumericMeasurement(Measurement):
    """
    Measurement of a numeric value
    """
    def __init__(self, value):
        self.value = value


Limit = namedtuple("Limit", "limit is_inclusive")


def get_inclusive_limit(value):
    """
    Gets an inclusive limit at the specified value
    >>> limit = get_inclusive_limit(-2)
    >>> limit.limit
    -2
    >>> limit.is_inclusive
    True
    """
    return Limit(value, True)


def get_exclusive_limit(value):
    """
    Gets an exclusive limit at the specified value
    >>> limit = get_exlusive_limit(-2)
    >>> limit.limit
    -2
    >>> limit.is_inclusive
    False
    """
    return Limit(value, False)


class LowerLimitCondition(Condition):
    """
    A condition for a numeric function with just a lower limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, lower_limit, name=""):
        Condition.__init__(self, name)
        self.lower_limit = lower_limit.limit
        self.lower_limit_is_inclusive = lower_limit.is_inclusive


class UpperLimitCondition(Condition):
    """
    A condition for a numeric function with just an upper limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, upper_limit, name=""):
        Condition.__init__(self, name)
        self.upper_limit = upper_limit.limit
        self.upper_limit_is_inclusive = upper_limit.is_inclusive


class BothLimitsCondition(LowerLimitCondition, UpperLimitCondition):
    """
    A condition for a numeric function with a lower limit and an upper limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, lower_limit, upper_limit, name=""):
        LowerLimitCondition.__init__(self, lower_limit, name)
        UpperLimitCondition.__init__(self, upper_limit, name)
