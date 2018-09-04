from rospit.framework import Evaluator, Evaluation, TestSuite, get_logger, Measurement
from rospit.declarative import Step
from rospit.binary import BinaryMeasurement
from rospit.numeric import Limit, LowerLimitCondition, LowerLimitEvaluator, \
                           UpperLimitCondition, UpperLimitEvaluator, \
                           BothLimitsCondition, BothLimitsEvaluator, \
                           GreaterThanCondition, GreaterThanEvaluator, \
                           GreaterThanOrEqualToCondition, GreaterThanOrEqualToEvaluator, \
                           EqualToCondition, EqualToEvaluator, \
                           NotEqualToCondition, NotEqualToEvaluator, \
                           LessThanCondition, LessThanEvaluator, \
                           LessThanOrEqualToCondition, LessThanOrEqualToEvaluator, \
                           NumericMeasurement
import rospy
import importlib
import time
import rosservice, genmsg, genpy
import std_msgs

def get_ros_type(type_string):
    parts = type_string.split("/")
    if len(parts) != 2:
        raise Exception("Type lookup requires two parts, split by a slash: Package and type")
    package = parts[0]
    ros_type = parts[1]
    return (package, ros_type)


def get_subscriber(topic, msg_type_str, func, params=None):
    package, ros_type = get_ros_type(msg_type_str)
    mod = importlib.import_module(package + ".msg")
    msg_type = getattr(mod, ros_type)
    if params is None:
        return rospy.Subscriber(topic, msg_type, func)
    else:
        return rospy.Subscriber(topic, msg_type, func, params)


class ROSTestSuite(TestSuite):
    def __init__(self, subscribers, name=""): 
        TestSuite.__init__(self, name)
        self.messages = dict()
        self.message_received_on = set()
        self.msg_value_subscribers = dict()
        self.msg_received_subscribers = dict()
        for topic, msg_type in subscribers.msg_value_subscribers:
            self.msg_value_subscribers[topic] = get_subscriber(topic, msg_type, self.store_message, topic)
        for topic, msg_type in subscribers.msg_received_subscribers:
            self.msg_received_subscribers[topic] = get_subscriber(topic, msg_type, self.store_msg_received_on, topic)

    def store_message(self, data, topic):
        self.messages[topic] = data

    def store_msg_received_on(self, data, topic):
        self.message_received_on.add(topic)

    
class MessageValue(object):
    def __init__(self, topic, field, test_suite):
        self.topic = topic
        self.field = field
        self.test_suite = test_suite

    def get_value(self):
        message = self.test_suite.messages[self.topic]
        return get_field_or_message(message, self.field)


def get_field_or_message(message, field_str):
    data = message
    if field_str is not None:
        fields = field_str.split("/")
        while len(fields) > 0:
            field = fields.pop(0)
            data = getattr(data, field)
    return data


class MessageEvaluatorBase(Evaluator):
    def __init__(self, topic, topic_type, field=None):
        package, topic_msg = get_ros_type(topic_type)
        mod = importlib.import_module(package + ".msg")
        obj = getattr(mod, topic_msg)
        self.subscriber = rospy.Subscriber(topic, obj, self.callback)
        self.received = False
        self.topic = topic
        self.field = field
        self.data = None

    def callback(self, data):
        self.data = get_field_or_message(data, self.field)
        self.received = True


class MessageReceivedEvaluator(MessageEvaluatorBase):
    """
    Evaluates whether a message has been received on the topic
    """
    def __init__(self, topic, topic_type, field=None):
        MessageEvaluatorBase.__init__(self, topic, topic_type, field)

    def evaluate(self, condition, measurement=None):
        if measurement is None:
            measurement = BinaryMeasurement(self.received)
        return Evaluation(measurement, condition, self.received == condition.value)


class MessageEvaluator(MessageEvaluatorBase):
    def __init__(self, topic, topic_type, field=None):
        MessageEvaluatorBase.__init__(self, topic, topic_type, field)

    def evaluate(self, condition, measurement=None):
        if measurement is None:
            measurement = self.data
        return Evaluation(measurement, condition, self.data == condition.value)


class ExecutionReturnedEvaluator(Evaluator):
    def __init__(self, test_case, field=None):
        self.test_case = test_case
        self.field = field

    def evaluate(self, condition, measurement=None):
        if measurement is None:
            current = self.test_case.execution_result
            if self.field is not None:
                fields = self.field.split("/")
                for field in fields:
                    current = getattr(current, field)
            measurement = Measurement(current)

        evaluation = Evaluation(measurement, condition, measurement.value == condition.value)
        get_logger().debug("Condition {}, measurement {}, {}".format(condition, measurement, "nominal" if evaluation.nominal else "not nominal"))
        return evaluation

        
class NumericMessageEvaluator(MessageEvaluatorBase):
    def __init__(self, topic, topic_type, field=None):
        MessageEvaluatorBase.__init__(self, topic, topic_type, field)

    def evaluate(self, condition, measurement=None):
        if measurement is None:
            while self.data is None:
                time.sleep(1)
            measurement = NumericMeasurement(self.data)
        # BothLimits should be above LowerLimit and UpperLimit as it is a child class
        type_map = {
            BothLimitsCondition: BothLimitsEvaluator,
            UpperLimitCondition: UpperLimitEvaluator,
            LowerLimitCondition: LowerLimitEvaluator,
            GreaterThanCondition: GreaterThanEvaluator,
            GreaterThanOrEqualToCondition: GreaterThanOrEqualToEvaluator,
            EqualToCondition: EqualToEvaluator,
            NotEqualToCondition: NotEqualToEvaluator,
            LessThanOrEqualToCondition: LessThanOrEqualToEvaluator,
            LessThanCondition: LessThanEvaluator
        }

        evaluator_type = None
        for key, value in type_map.iteritems():
            if isinstance(condition, key):
                evaluator_type = value
                break
        if evaluator_type is None:
            raise ValueError("Condition is of unknown type")

        evaluation = evaluator_type(lambda: self.data).evaluate(condition, measurement)

        get_logger().info("Condition {}, measurement {}, {}".format(condition, measurement, "nominal" if evaluation.nominal else "not nominal"))

        return evaluation


class ServiceCall(Step):
    """
    A call to a ROS Service
    """
    def __init__(self, service, service_type, parameters=None, msg_type=None, msg_args=None, save_result=False):
        Step.__init__(self, save_result)
        if parameters is None:
            parameters = dict()
        self.service = service
        self.service_type = service_type
        self.parameters = parameters 

    def execute(self):
        return call_service(self.service, _fill_parameters(self.parameters))[1]


def call_service(service_name, service_args):
    """
    Call a service. Mostly extracted from rosservice.
    """
    service_class = rosservice.get_service_class_by_name(service_name)
    request = service_class._request_class()
    try:
        now = rospy.get_rostime()
        keys = { 'now': now, 'auto': std_msgs.msg.Header(stamp=now) }
        genpy.message.fill_message_args(request, service_args, keys=keys)
    except genpy.MessageException as e:
        def argsummary(args):
            if type(args) in [tuple, list]:
                return '\n'.join([' * %s (type %s)'%(a, type(a).__name__) for a in args])
            else:
                return ' * %s (type %s)'%(args, type(args).__name__)
            raise rosservice.ROSServiceException("Incompatible arguments to call service:\n%s\nProvided arguments are:\n%s\n\nService arguments are: [%s]"%(e, argsummary(service_args), genpy.message.get_printable_message_args(request)))
    try:
        return request, rospy.ServiceProxy(service_name, service_class)(request)
    except rospy.ServiceException as e:
        raise rosservice.ROSServiceException(str(e))
    except genpy.SerializationError as e:
        raise rosservice.ROSServiceException("Unable to send request. One of the fields has an incorrect type:\n"+\
                                             "  %s\n\nsrv file:\n%s"%(e, rosmsg.get_srv_text(service_class._type)))
    except rospy.ROSSerializationException as e:
        raise rosservice.ROSServiceException("Unable to send request. One of the fields has an incorrect type:\n"+\
                                             "  %s\n\nsrv file:\n%s"%(e, rosmsg.get_srv_text(service_class._type)))


def _fill_parameters(parameters):
    def _process(item):
        if isinstance(item, dict):
            return_value = dict()
            for key, value in item.iteritems():
                return_value[key] = _process(value)
            return return_value
        elif isinstance(item, MessageValue):
            return item.get_value()
        else:
            return item
    val = _process(parameters)
    return [val]


class Sleep(Step):
    """
    A call to ROS sleep
    """
    def __init__(self, time, unit="second"):
        Step.__init__(self, False)
        if unit == "second" or unit == "seconds":
            self.time = time
        elif unit == "minute" or unit == "minutes":
            self.time = time * 60
        elif unit == "hour" or unit == "hours":
            self.time = time * 60 * 60
        else:
            raise ValueError("Supported units are second, minute and hour")

    def execute(self):
        rospy.sleep(float(self.time))
