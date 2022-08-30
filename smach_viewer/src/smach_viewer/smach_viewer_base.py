#!/usr/bin/env python

import threading

import base64
import pickle
import roslib
import rospy
import smach
import smach_ros

from smach_viewer.text_wrapper import TextWrapper
from smach_viewer.utils import attr_string
from smach_viewer.utils import graph_attr_string

from smach_msgs.msg import SmachContainerStatus
from smach_msgs.msg import SmachContainerStructure


class ContainerNode(object):
    """
    This class represents a given container in a running SMACH system.

    Its primary use is to generate dotcode for a SMACH container. It has
    methods for responding to structure and status messages from a SMACH
    introspection server, as well as methods for updating the styles of a
    graph once it's been drawn.
    """
    def __init__(self, server_name, msg):
        # Store path info
        self._server_name = server_name
        self._path = msg.path
        splitpath = msg.path.split('/')
        self._label = splitpath[-1]
        self._dir = '/'.join(splitpath[0:-1])

        self._children = msg.children
        self._internal_outcomes = msg.internal_outcomes
        self._outcomes_from = msg.outcomes_from
        self._outcomes_to = msg.outcomes_to

        self._container_outcomes = msg.container_outcomes

        # Status
        self._initial_states = []
        self._active_states = []
        self._last_active_states = []
        self._local_data = smach.UserData()
        self._info = ''

    def update_structure(self, msg):
        """Update the structure of this container from a given message. Return True if anything changes."""
        needs_update = False

        if self._children != msg.children\
                or self._internal_outcomes != msg.internal_outcomes\
                or self._outcomes_from != msg.outcomes_from\
                or self._outcomes_to != msg.outcomes_to\
                or self._container_outcomes != msg.container_outcomes:
            needs_update = True

        if needs_update:
            self._children = msg.children
            self._internal_outcomes = msg.internal_outcomes
            self._outcomes_from = msg.outcomes_from
            self._outcomes_to = msg.outcomes_to

            self._container_outcomes = msg.container_outcomes

        return needs_update

    def _load_local_data(self, msg):
        """Unpack the user data"""
        try:
            local_data = pickle.loads(msg.local_data)
        except:
            if isinstance(msg.local_data, str):
                local_data = pickle.loads(base64.b64decode(msg.local_data))
            else:
                local_data = pickle.loads(base64.b64decode(bytes(str(msg.local_data).encode('utf-8'))))
        return local_data

    def update_status(self, msg):
        """Update the known userdata and active state set and return True if the graph needs to be redrawn."""

        # Initialize the return value
        needs_update = False

        # Check if the initial states or active states have changed
        if set(msg.initial_states) != set(self._initial_states):
            self._structure_changed = True
            needs_update = True
        if set(msg.active_states) != set(self._active_states):
            needs_update = True

        # Store the initial and active states
        self._initial_states = msg.initial_states
        self._last_active_states = self._active_states
        self._active_states = msg.active_states

        # Unpack the user data
        while not rospy.is_shutdown():
            try:
                self._local_data._data = self._load_local_data(msg)
                break
            except ImportError as ie:
                # This will only happen once for each package
                modulename = ie.args[0][16:]
                packagename = modulename[0:modulename.find('.')]
                roslib.load_manifest(packagename)
                self._local_data._data = self._load_local_data(msg)

        # Store the info string
        self._info = msg.info

        return needs_update

    def get_dotcode(self, selected_paths, closed_paths, depth, max_depth, containers, show_all, label_wrapper, attrs={}):
        """Generate the dotcode representing this container.

        @param selected_paths: The paths to nodes that are selected
        @closed paths: The paths that shouldn't be expanded
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to which we should traverse the tree
        @param containers: A dict of containers keyed by their paths
        @param show_all: True if implicit transitions should be shown
        @param label_wrapper: A text wrapper for wrapping element names
        @param attrs: A dict of dotcode attributes for this cluster
        """

        dotstr = 'subgraph "cluster_%s" {\n' % (self._path)
        if depth == 0:
            #attrs['style'] = 'filled,rounded'
            attrs['color'] = '#00000000'
            attrs['fillcolor'] = '#0000000F'
        #attrs['rank'] = 'max'

        #,'succeeded','aborted','preempted'attrs['label'] = self._label
        dotstr += graph_attr_string(attrs)

        # Add start/terimate target
        proxy_attrs = {
                'URL':self._path,
                'shape':'plaintext',
                'color':'gray',
                'fontsize':'18',
                'fontweight':'18',
                'rank':'min',
                'height':'0.01'}
        proxy_attrs['label'] = '\\n'.join(label_wrapper.wrap(self._label))
        dotstr += '"%s" %s;\n' % (
                '/'.join([self._path,'__proxy__']),
                attr_string(proxy_attrs))

        # Check if we should expand this container
        if max_depth == -1 or depth <= max_depth:
            # Add container outcomes
            dotstr += 'subgraph "cluster_%s" {\n' % '/'.join([self._path,'__outcomes__'])
            outcomes_attrs = {
                    'style':'rounded,filled',
                    'rank':'sink',
                    'color':'#FFFFFFFF',#'#871C34',
                    'fillcolor':'#FFFFFF00'#'#FE464f3F'#'#DB889A'
                    }
            dotstr += graph_attr_string(outcomes_attrs)

            for outcome_label in self._container_outcomes:
                outcome_path = ':'.join([self._path,outcome_label])
                outcome_attrs = {
                        'shape':'box',
                        'height':'0.3',
                        'style':'filled,rounded',
                        'fontsize':'12',
                        'fillcolor':'#FE464f',#'#EDC2CC',
                        'color':'#780006',#'#EBAEBB',
                        'fontcolor':'#780006',#'#EBAEBB',
                        'label':'',
                        'xlabel':'\\n'.join(label_wrapper.wrap(outcome_label)),
                        'URL':':'.join([self._path,outcome_label])
                        }
                dotstr += '"%s" %s;\n' % (outcome_path,attr_string(outcome_attrs))
            dotstr += "}\n"

            # Iterate over children
            for child_label in self._children:
                child_path = '/'.join([self._path,child_label])
                # Generate dotcode for children
                if child_path in containers:
                    if child_label in self._active_states:
                        child_color = '#5C7600FF'
                        child_fillcolor = '#C0F700FF'
                        child_linewidth = 5
                    else:
                        child_color = '#000000FF'
                        child_fillcolor = 'gray'
                        child_linewidth = 2

                    child_attrs = {
                        'style': 'filled,setlinewidth({}),rounded'.format(child_linewidth),
                        'color': child_color,
                        'fillcolor': child_fillcolor,
                    }

                    dotstr += containers[child_path].get_dotcode(
                        selected_paths,
                        closed_paths,
                        depth+1, max_depth,
                        containers,
                        show_all,
                        label_wrapper,
                        child_attrs
                    )
                else:
                    if child_label in self._active_states:
                        child_color = '#5C7600FF'
                        child_fillcolor = '#C0F700FF'
                        child_linewidth = 5
                    else:
                        child_color = '#000000FF'
                        child_fillcolor = '#FFFFFFFF'
                        child_linewidth = 2

                    child_attrs = {
                        'style': 'filled,setlinewidth({})'.format(child_linewidth),
                        'color': child_color,
                        'fillcolor': child_fillcolor,
                        'label': '\\n'.join(label_wrapper.wrap(child_label)),
                        'URL': child_path,
                    }
                    dotstr += '"%s" %s;\n' % (child_path, attr_string(child_attrs))

            # Iterate over edges
            internal_edges = list(zip(
                    self._internal_outcomes,
                    self._outcomes_from,
                    self._outcomes_to))

            # Add edge from container label to initial state
            internal_edges += [('','__proxy__',initial_child) for initial_child in self._initial_states]

            has_explicit_transitions = []
            for (outcome_label,from_label,to_label) in internal_edges:
                if to_label != 'None' or outcome_label == to_label:
                    has_explicit_transitions.append(from_label)

            # Draw internal edges
            for (outcome_label,from_label,to_label) in internal_edges:

                from_path = '/'.join([self._path, from_label])

                if show_all \
                        or to_label != 'None'\
                        or from_label not in has_explicit_transitions \
                        or (outcome_label == from_label) \
                        or from_path in containers:
                    # Set the implicit target of this outcome
                    if to_label == 'None':
                        to_label = outcome_label

                    to_path = '/'.join([self._path, to_label])

                    edge_attrs = {
                            'URL':':'.join([from_path,outcome_label,to_path]),
                            'fontsize':'12',
                            'label':'',
                            'xlabel':'\\n'.join(label_wrapper.wrap(outcome_label))}
                    edge_attrs['style'] = 'setlinewidth(2)'

                    # Hide implicit
                    #if not show_all and to_label == outcome_label:
                    #    edge_attrs['style'] += ',invis'

                    from_key = '"%s"' % from_path
                    if from_path in containers:
                        if max_depth == -1 or depth+1 <= max_depth:
                            from_key = '"%s:%s"' % ( from_path, outcome_label)
                        else:
                            edge_attrs['ltail'] = 'cluster_'+from_path
                            from_path = '/'.join([from_path,'__proxy__'])
                            from_key = '"%s"' % ( from_path )

                    to_key = ''
                    if to_label in self._container_outcomes:
                        to_key = '"%s:%s"' % (self._path,to_label)
                        edge_attrs['color'] = '#00000055'# '#780006'
                    else:
                        if to_path in containers:
                            edge_attrs['lhead'] = 'cluster_'+to_path
                            to_path = '/'.join([to_path,'__proxy__'])
                        to_key = '"%s"' % to_path

                    dotstr += '%s -> %s %s;\n' % (
                            from_key, to_key, attr_string(edge_attrs))

        dotstr += '}\n'
        return dotstr


class SmachViewerBase(object):

    _container_class = ContainerNode

    def __init__(self):
        # Create graph
        self._containers = {}
        self._top_containers = {}
        self._update_cond = threading.Condition()
        self._needs_refresh = True
        self.dotstr = ''

        # smach introspection client
        self._client = smach_ros.IntrospectionClient()
        self._containers = {}
        self._selected_paths = []

        # Message subscribers
        self._structure_subs = {}
        self._status_subs = {}

        self._path = '/'
        self._needs_zoom = True
        self._structure_changed = True
        self._max_depth = -1
        self._show_all_transitions = False
        self._label_wrapper = TextWrapper(40, break_long_words=True)

        # Start a thread in the background to update the server list
        self._keep_running = True
        self._server_list_thread = threading.Thread(
            target=self._update_server_list)
        self._server_list_thread.start()

        self._update_graph_thread = threading.Thread(target=self._update_graph)
        self._update_graph_thread.start()

    def _update_graph(self):
        """This thread continuously updates the graph when it changes.

        The graph gets updated in one of two ways:

          1: The structure of the SMACH plans has changed, or the display
          settings have been changed. In this case, the dotcode needs to be
          regenerated.

          2: The status of the SMACH plans has changed. In this case, we only
          need to change the styles of the graph.
        """
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                self._update_graph_step()

    def _update_graph_step(self):
        # Wait for the update condition to be triggered
        self._update_cond.wait()

        containers_to_update = {}
        if self._path in self._containers:
            # Some non-root path
            containers_to_update = {
                self._path: self._containers[self._path]}
        elif self._path == '/':
            # Root path
            containers_to_update = self._top_containers

        # Check if we need to re-generate the dotcode
        # (if the structure changed)
        # TODO(???): needs_zoom is a misnomer
        if self._structure_changed or self._needs_zoom:
            dotstr = "digraph {\n\t"
            dotstr += ';'.join([
                "compound=true",
                "outputmode=nodesfirst",
                "labeljust=l",
                "nodesep=0.5",
                "minlen=2",
                "mclimit=5",
                "clusterrank=local",
                "ranksep=0.75",
                # "remincross=true",
                # "rank=sink",
                "ordering=\"\"",
                ])
            dotstr += ";\n"

            # Generate the rest of the graph
            # TODO(???): Only re-generate dotcode
            # for containers that have changed
            for path, tc in containers_to_update.items():
                dotstr += tc.get_dotcode(
                    self._selected_paths, [],
                    0, self._max_depth,
                    self._containers,
                    self._show_all_transitions,
                    self._label_wrapper)
            if len(containers_to_update) == 0:
                dotstr += '"__empty__" ' \
                    + '[label="Path not available.", shape="plaintext"]'

            dotstr += '\n}\n'
            self.dotstr = dotstr
        return containers_to_update

    def _update_server_list(self):
        """Update the list of known SMACH introspection servers."""
        while self._keep_running:
            # Update the server list
            server_names = self._client.get_servers()
            new_server_names = [
                sn for sn in server_names if sn not in self._status_subs]

            # Create subscribers for new servers
            for server_name in new_server_names:
                self._structure_subs[server_name] = rospy.Subscriber(
                        server_name+smach_ros.introspection.STRUCTURE_TOPIC,
                        SmachContainerStructure,
                        callback=self._structure_msg_update,
                        callback_args=server_name,
                        queue_size=50)

                self._status_subs[server_name] = rospy.Subscriber(
                        server_name+smach_ros.introspection.STATUS_TOPIC,
                        SmachContainerStatus,
                        callback=self._status_msg_update,
                        queue_size=50)

            # This doesn't need to happen very often
            rospy.sleep(1.0)

    def _structure_msg_update(self, msg, server_name):
        """Update the structure of the SMACH plan (re-generate the dotcode)."""

        # Just return if we're shutting down
        if not self._keep_running:
            return

        # Get the node path
        path = msg.path
        pathsplit = path.split('/')
        parent_path = '/'.join(pathsplit[0:-1])

        rospy.logdebug("RECEIVED: "+path)
        rospy.logdebug("CONTAINERS: "+str(list(self._containers.keys())))

        # Initialize redraw flag
        needs_redraw = False

        if path in self._containers:
            rospy.logdebug("UPDATING: "+path)

            # Update the structure of this known container
            needs_redraw = self._containers[path].update_structure(msg)
        else:
            rospy.logdebug("CONSTRUCTING: "+path)

            # Create a new container
            container = self._container_class(server_name, msg)
            self._containers[path] = container

            # Store this as a top container if it has no parent
            if parent_path == '':
                self._top_containers[path] = container

            # We need to redraw thhe graph
            # if this container's parent is already known
            if parent_path in self._containers:
                needs_redraw = True

        # Update the graph if necessary
        if needs_redraw:
            with self._update_cond:
                self._structure_changed = True
                # TODO(???): Make it so you can disable this
                self._needs_zoom = True
                self._update_cond.notify_all()

    def _status_msg_update(self, msg):
        """Process status messages."""

        # Check if we're in the process of shutting down
        if not self._keep_running:
            return

        # Get the path to the updating conainer
        path = msg.path
        rospy.logdebug("STATUS MSG: "+path)

        # Check if this is a known container
        if path in self._containers:
            # Get the container and check
            # if the status update requires regeneration
            container = self._containers[path]
            if container.update_status(msg):
                with self._update_cond:
                    self._update_cond.notify_all()

    def kill(self):
        with self._update_cond:
            self._keep_running = False
            self._update_cond.notify_all()

        self._server_list_thread.join()
        self._update_graph_thread.join()
