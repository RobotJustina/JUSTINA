import re

import BB
from messages import Message, MessageTypes, Command, Response

class SharedVarTypes(object):
    '''
    Pseudo-enum of the types of shared variables.
    
    The existing values are:
    
        * SharedVarTypes.BYTE_ARRAY
        * SharedVarTypes.INT
        * SharedVarTypes.INT_ARRAY
        * SharedVarTypes.LONG
        * SharedVarTypes.LONG_ARRAY
        * SharedVarTypes.DOUBLE
        * SharedVarTypes.DOUBLE_ARRAY
        * SharedVarTypes.STRING
        * SharedVarTypes.MATRIX
        * SharedVarTypes.RECOGNIZED_SPEECH
        * SharedVarTypes.VAR
    
    The type VAR can be considered as an "unknown" type. It doesn't do any special serialization or deserialization processing,
    instead it sends the string the user inputs as parameters directly. 
    '''
    
    BYTE_ARRAY = 'byte[]'
    INT = 'int'
    INT_ARRAY = 'int[]'
    LONG = 'long'
    LONG_ARRAY = 'long[]'
    DOUBLE = 'double'
    DOUBLE_ARRAY = 'double[]'
    STRING = 'string'
    MATRIX = 'matrix'
    RECOGNIZED_SPEECH = 'RecognizedSpeech'
    VAR = 'var'

class SubscriptionTypes(object):
    '''
    Pseudo-enum of the types of subscriptions. (See :meth:`SubscribeToSharedVar <pyRobotics.BB.SubscribeToSharedVar>`)
    
    The existing values are:
    
        * SubscriptionTypes.CREATION
        * SubscriptionTypes.WRITE_MODULE
        * SubscriptionTypes.WRITE_OTHERS
        * SubscriptionTypes.WRITE_ANY
        
    '''
    
    CREATION = 'creation'
    WRITE_MODULE = 'writemodule'
    WRITE_OTHERS = 'writeothers'
    WRITE_ANY = 'writeany'
    
class ReportTypes(object):
    '''
    Pseudo-enum of the types of reports. (See :meth:`SubscribeToSharedVar <pyRobotics.BB.SubscribeToSharedVar>`)
    
    The existing values are:
    
        * ReportTypes.CONTENT
        * ReportTypes.NOTIFY
    '''
    CONTENT = 'content'
    NOTIFY = 'notify'

class SharedVar(Message):
    '''
    A wrapper for the shared variable notifications.
    
    An object of this type is passed as parameter to every subscription handler function. (See :meth:`SubscribeToSharedVar <pyRobotics.BB.SubscribeToSharedVar>`)
    
    Objects of this class include the members:
    
    sv.varName
        The name of the shared variable of which a notification was received.
    sv.svType
        One of the class variables in the pseudo-enum :class:`SharedVarTypes`.
    sv.size
        -1 if it is not an array type, an integer if sv.svType is of type SharedVariableTypes.BYTE_ARRAY,
        SharedVariableTypes.INT_ARRAY, SharedVariableTypes.LONG_ARRAY or SharedVariableTypes.DOUBLE_ARRAY.
    sv.data
        Contains the deserialized data of this shared variable, depending on its type.
    
    '''
    _rx = re.compile(r'^\s*({\s*)?(?P<type>([a-zA-Z_][_a-zA-Z0-9]*))(?P<array>(\[(?P<size>\d+)?\]))?\s+(?P<name>([a-zA-Z_][_a-zA-Z0-9]*))\s+(?P<data>(("(\\.|[^"])*")|({[^}]*})))?\s*}?((\s+%)?\s+(?P<report>(\w+))\s+%\s+(?P<subscription>(\w+))\s+%\s+(?P<writer>([A-Z][0-9A-Z\-]*)))?')
    
    def __init__(self, responseObj):
        super(SharedVar, self).__init__(responseObj.name)
        self.type = MessageTypes.SHARED_VAR
        self.successful = responseObj.successful
        self.params = responseObj.params
        self._id = responseObj._id
    
    @classmethod
    def Parse(cls, s):
        r = Response.Parse(s)
        if not (r and r.name == 'read_var'):
            return r
        
        var = SharedVar(r)
        
        m = SharedVar._rx.match(var.params)
        
        if not m:
            print 'read_var received but failed to parse:'
            print var.params
            return None
        
        var.svType = m.group('type') + ('[]' if m.group('array') else '')
        var.size = -1 if not m.group('size') else int(m.group('size'))
        var.varName = m.group('name')
        var.data = m.group('data')
        
        var.report = m.group('report')
        var.subscription = m.group('subscription')
        var.writer = m.group('writer')
        
        var.isNotification = not not var.report
        
        var.data = SharedVar._ProcessReadVar(var)
        
        return var
    
    @classmethod
    def _ProcessReadVar(cls, var):
    
        try:
            if not var.data:
                return None
            
            if var.svType == SharedVarTypes.STRING:
                return Message._DeserializeString(var.data)
            
            if var.svType in [SharedVarTypes.INT, SharedVarTypes.LONG]:
                return int(var.data)
            
            if var.svType == SharedVarTypes.DOUBLE:
                return float(var.data)
            
            if var.svType in [SharedVarTypes.INT_ARRAY, SharedVarTypes.LONG_ARRAY]:
                return [int(x) for x in var.data.split()]
            
            if var.svType == SharedVarTypes.DOUBLE_ARRAY:
                return [float(x) for x in var.data.split()]
            
            if var.svType == SharedVarTypes.BYTE_ARRAY:
                return SharedVar._DeserializeByteArray(var.data)
            
            if var.svType == SharedVarTypes.RECOGNIZED_SPEECH:
                return SharedVar._DeserializeRecognizedSpeech(var.data)
            
            if var.svType == SharedVarTypes.MATRIX:
                return SharedVar._DeserializeMatrix(var.data)
            
            if var.svType == SharedVarTypes.VAR:
                if var.data == 'null':
                    return None
                return var.data
        except:
            pass
        
        print 'Error parsing type: ' + var.svType
        return None
    
    @classmethod
    def _DeserializeByteArray(cls, data):
        data = data[2:]
        l = []
        while data:
            l.append(int(data[:2], 16))
            data = data[2:]
        return l

    @classmethod
    def _DeserializeMatrix(cls, data):
        rows, data = data.split(None, 1)
        x = rows.find('x')
        columns = int(rows[x+1:])
        rows = int(rows[:x])
        
        matrix = []
        
        for _ in range(rows):
            l = []
            for _ in range(columns):
                x = data.find(' ')
                if x > -1:
                    item, data = data.split(None, 1)
                else:
                    item = data
                l.append(float(item))
            matrix.append(l)
            
        return matrix

    @classmethod
    def _SerializeMatrix(cls, data):
        
        rows = len(data)
        cols = len(data[0])
        
        txt = str(rows) + 'x' + str(cols)
        
        for r in data:
            for c in r:
                txt += ' ' + str(c)
            
        return txt

    @classmethod
    def _DeserializeRecognizedSpeech(cls, data):
        '''Returns a list which contains tuples (2 elements each) with string and confidence.'''
        if data == '' or data == 'null':
            return None
        
        if data[0] != '{' or data[-1] != '}':
            return None
        
        data = data[1:-1].strip()
        count = 0
        
        if data.find(' ') > -1:
            count, data = data.split(None, 1)
        else:
            return None
        
        count = int(count)
        l = []
        for _ in range(count):
            if data[0] != '"':
                #REVISAR!!!!!!!!!!!
                return None
            data = data[1:]
            pos = data.find('"')
            if pos < 0:
                return None
            currentText = data[:pos]
            data = data[pos+1:].strip()
            if data.find(' ') > -1:
                currentConfidence, data = data.split(None, 1)
            else:
                currentConfidence = data
            currentConfidence = float(currentConfidence)
            l.append((currentText, currentConfidence))
        
        return l

def _CreateSharedVar(svType, name):
    r = BB.SendAndWait(Command('create_var', svType + ' ' + name) , 2000, 2)
    
    return (r and r.successful)
    
def _WriteSharedVar(sharedVarType, name, data):
    
    w = str(data)
    if sharedVarType == SharedVarTypes.BYTE_ARRAY:
        w = '0x' + ''.join([ "%02X" % x for x in data ])
    elif sharedVarType in [SharedVarTypes.INT, SharedVarTypes.LONG]:
        w = str(int(data))
    elif sharedVarType == SharedVarTypes.DOUBLE:
        w = str(float(data))
    elif sharedVarType in [SharedVarTypes.INT_ARRAY, SharedVarTypes.LONG_ARRAY]:
        w = ' '.join([str(int(x)) for x in data])
    elif sharedVarType == SharedVarTypes.DOUBLE_ARRAY:
        w = ' '.join([str(float(x)) for x in data])
    elif sharedVarType == SharedVarTypes.STRING:
        w = Message._SerializeString(data)
    elif sharedVarType == SharedVarTypes.MATRIX:
        w = SharedVar._SerializeMatrix(data)
    else:
        print 'pyRobotics - ERROR: Unhandled shared var type'
        return False
    
    r = BB.SendAndWait(Command('write_var', sharedVarType + ' ' + name + ' ' + w), 2000, 2)
    
    return (r and r.successful)

def _ReadSharedVar(name):
    
    r = BB.SendAndWait(Command('read_var', name), 2000)
    
    if not (r and r.successful):
        return None
    
    return r.data

def _SubscribeToSharedVar(name, subscriptionType, reportType):
    
    r = BB.SendAndWait(Command('suscribe_var', name + ' suscribe=' + subscriptionType + ' report=' + reportType), 2000, 2)
    return (r and r.successful)
