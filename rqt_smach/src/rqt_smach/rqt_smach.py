import os
import rospy
import roslib
import sys
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtGui import QStyle,QApplication,QMouseEvent
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal,Slot
from python_qt_binding.QtGui import QWidget,QPalette,QColor,QStandardItemModel,QItemDelegate,QStyleOptionButton,QStandardItem,QIcon
from python_qt_binding.QtCore import Qt,QTime,QTimer,Signal,QRect,QSize,QEvent
from rqt_py_common.extended_combo_box import ExtendedComboBox
from qt_gui_py_common.worker_thread import WorkerThread

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import threading
import pickle
import pprint
import copy
import StringIO
import colorsys
import textwrap
import xdot
import smach
import smach_ros

from xdot.xdot_qt import DotWidget
import std_msgs.msg
import actionlib
import smach_msgs.msg
from actionlib_msgs.msg import GoalStatus
from smach_msgs.msg import SmachContainerStatus,SmachContainerInitialStatusCmd,SmachContainerStructure
from .container_node import ContainerNode

### Helper Functions
def graph_attr_string(attrs):
    """Generate an xdot graph attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ';\n'.join(attrs_strs)+';\n'

def attr_string(attrs):
    """Generate an xdot node attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ' ['+(', '.join(attrs_strs))+']'

def get_parent_path(path):
    """Get the parent path of an xdot node."""
    path_tokens = path.split('/')
    if len(path_tokens) > 2:
        parent_path = '/'.join(path_tokens[0:-1])
    else:
        parent_path = '/'.join(path_tokens[0:1])
    return parent_path

def get_label(path):
    """Get the label of an xdot node."""
    path_tokens = path.split('/')
    return path_tokens[-1]

def hex2t(color_str):
    """Convert a hexadecimal color strng into a color tuple."""
    color_tuple = [int(color_str[i:i+2],16)/255.0    for i in range(1,len(color_str),2)]
    return color_tuple

class BlocksDelegate(QItemDelegate):
    def __init__(self, smach=None, parent=None):
        super(BlocksDelegate, self).__init__(parent)
        self.smach = smach

    def sizeHint(self, option, index):
        if 0 and index.column() == 0:
            return QSize(20, option.rect().height())

        return QSize()

    def editorEvent(self, event, model, option, index):
        success = super(BlocksDelegate, self).editorEvent(event, model, option, index)
        if event.type() == QEvent.MouseButtonRelease:
            self.smach._update_blocks()

        return success

class GroupsDelegate(QItemDelegate):
    def __init__(self, smach=None, parent=None):
        super(GroupsDelegate, self).__init__(parent)
        self.smach = smach

    def paint(self, painter, option, index):
        if index.column() in [0,3]:
            return super(GroupsDelegate, self).paint(painter, option, index)

        button = QStyleOptionButton()
        r = option.rect
        x = r.left() + r.width() - 30
        y = r.top()+ 2
        w = 28
        h = 14
        button.rect = QRect(x,y,w,h)
        button.text = '+' if index.column() == 1 else '-'
        button.state = QStyle.State_Enabled

        QApplication.style().drawControl(QStyle.CE_PushButton, button, painter)

    def editorEvent(self, event, model, option, index):
        success = super(GroupsDelegate, self).editorEvent(event, model, option, index)
        if event.type() == QEvent.MouseButtonRelease:
            if index.column() in [1,2]:

                #QMouseEvent ev

                click_x = event.x()
                click_y = event.x()

                if index.column() == 1:
                    self.smach._enable_group(index.row(),True)
                elif index.column() == 2:
                    self.smach._enable_group(index.row(),False)

        return success


class Smach(Plugin):
    update_graph_sig = Signal(str)

    def __init__(self, context):
        super(Smach, self).__init__(context)

        self.initialized = 0

        self._dotcode_sub = None
        self._topic_dict = {}
        self._update_thread = WorkerThread(self._update_thread_run, self._update_finished)

        # Give QObjects reasonable names
        self.setObjectName('Smach')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()

        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'rqt_smach.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.ns_refresh_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.setObjectName('SmachPluginUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        palette = QPalette ()
        palette.setColor(QPalette.Background, Qt.white)
        self._widget.setPalette(palette)

        #TODO do for both combo boxes
        self._widget.namespace_input.currentIndexChanged.connect(self._handle_ns_changed)
        self._widget.ns_refresh_button.clicked.connect(self.refresh_combo_box)
        self._widget.restrict_ns.stateChanged.connect(self.refresh_combo_box)
        self._widget.ud_path_input.currentIndexChanged.connect(self._handle_ud_path)
        self._widget.ud_set_initial.clicked.connect(self._handle_ud_set_path)
        self._widget.ud_text_browser.setReadOnly(1)
        #TODO if unchecked do handle_ns_changed

        self._widget.tree.setColumnCount(1)
        self._widget.tree.setHeaderLabels(["Containers"])

        self._ns = ""
        self.refresh_combo_box()

        # Bind path list
        self._widget.path_input.currentIndexChanged.connect(
                self._handle_path_changed)

        #Keep Combo Boxes sorted
        self._widget.namespace_input.setInsertPolicy(6)
        self._widget.path_input.setInsertPolicy(6)
        self._widget.ud_path_input.setInsertPolicy(6)

        # Create graph data structures
        # Containers is a map of (container path) -> container proxy
        self._containers = {}
        self._top_containers = {}

        # smach introspection client
        self._client = smach_ros.IntrospectionClient()
        self._selected_paths = []

        # Message subscribers
        self._structure_subs = {}
        self._status_subs = {}

        # Initialize xdot display state
        self._path = '/'
        self._needs_zoom = True
        self._structure_changed = True
        self._max_depth = -1
        self._show_all_transitions = False
        self._label_wrapper = textwrap.TextWrapper(40,break_long_words=True)
        self._graph_needs_refresh = True
        self._tree_needs_refresh = True


        self._keep_running = True

        self._update_server_list()
        self._update_graph()
        self._update_tree()

        # Start a timer to update the server list
        self._server_timer = QTimer(self)
        self._server_timer.timeout.connect(self._update_server_list)
        self._server_timer.start(1000)

        # Start a timer to update the graph display
        self._graph_timer = QTimer(self)
        self._graph_timer.timeout.connect(self._update_graph)
        self._graph_timer.start(1093)

        # Start a timer to update the._widget.tree display
        self._tree_timer = QTimer(self)
        self._tree_timer.timeout.connect(self._update_tree)
        self._tree_timer.start(1217)

        self._widget.tree.show()

    def _handle_ud_set_path(self):
        """Event: Set a sInitial Stae Button Pressed."""
        self._widget.path_input.setCurrentIndex(self._widget.path_input.findText(self._widget.ud_path_input.currentText()))
        self._path = self._widget.path_input.currentText()
        self._graph_needs_refresh = True

    def _handle_ud_path(self):
        """Event: User Data selection combo box changed"""
        path_input_str = self._widget.ud_path_input.currentText()
        #Check the path is non-zero length
        if len(path_input_str) > 0:
            # Split the path (state:outcome), and get the state path
            path = path_input_str.split(':')[0]

            # Get the container corresponding to this path, since userdata is
            # stored in the containers
            if path not in self._containers:
                parent_path = get_parent_path(path)
            else:
                parent_path = path

            if parent_path in self._containers:
                # Get the container
                container = self._containers[path_input_str]

                # Store the scroll position and selection
                #pos = self.ud_txt.HitTestPos(wx.Point(0,0))
                #sel = self.ud_txt.GetSelection()

                # Generate the userdata string
                ud_str = ''
                for (k,v) in container._local_data._data.iteritems():
                    ud_str += str(k)+": "
                    vstr = str(v)
                    # Add a line break if this is a multiline value
                    if vstr.find('\n') != -1:
                        ud_str += '\n'
                    ud_str+=vstr+'\n\n'
                #Display the user data
                self._widget.ud_text_browser.setPlainText(ud_str)
                self._widget.ud_text_browser.show()

    def _update_server_list(self):
        """Update the list of known SMACH introspection servers."""
        # Discover new servers
        if self._widget.restrict_ns.isChecked():
            server_names = [self._widget.namespace_input.currentText()[0:-1]]
            #self._status_subs = {}
        else:
            server_names = self._client.get_servers()

        new_server_names = [sn for sn in server_names if sn not in self._status_subs]

        # Create subscribers for newly discovered servers
        for server_name in new_server_names:

            # Create a subscriber for the plan structure (topology) published by this server
            self._structure_subs[server_name] = rospy.Subscriber(
                    server_name+smach_ros.introspection.STRUCTURE_TOPIC,
                    SmachContainerStructure,
                    callback = self._structure_msg_update,
                    callback_args = server_name,
                    queue_size=50)

            # Create a subscriber for the active states in the plan published by this server
            self._status_subs[server_name] = rospy.Subscriber(
                    server_name+smach_ros.introspection.STATUS_TOPIC,
                    SmachContainerStatus,
                    callback = self._status_msg_update,
                    queue_size=50)

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
        rospy.logdebug("CONTAINERS: "+str(self._containers.keys()))

        # Initialize redraw flag
        needs_redraw = False

        # Determine if we need to update one of the proxies or create a new one
        if path in self._containers:
            rospy.logdebug("UPDATING: "+path)

            # Update the structure of this known container
            needs_redraw = self._containers[path].update_structure(msg)
        else:
            rospy.logdebug("CONSTRUCTING: "+path)

            # Create a new container
            container = ContainerNode(server_name, msg)
            self._containers[path] = container

            # Store this as a top container if it has no parent
            if parent_path == '':
                self._top_containers[path] = container

            # Append the path to the appropriate widgets
            self._append_new_path(path)

            # We need to redraw the graph if this container's parent is already known
            if parent_path in self._containers:
                needs_redraw = True

        if needs_redraw:
            self._structure_changed = True
            self._needs_zoom = True # TODO: Make it so you can disable this
            self._tree_needs_refresh = True
            self._graph_needs_refresh = True

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
            # Get the container and check if the status update requires regeneration
            container = self._containers[path]
            if container.update_status(msg):
                self._graph_needs_refresh = True
                self._tree_needs_refresh = True

            # TODO: Is this necessary?
            """path_input_str = self.path_input.GetValue()
            if path_input_str == path or get_parent_path(path_input_str) == path:
                wx.PostEvent(
                        self.path_input.GetEventHandler(),
                        wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED,self.path_input.GetId()))
            """

    def _append_new_path(self, path):
        """Append a new path to the path selection widgets"""
        if ((not self._widget.restrict_ns.isChecked()) or ((self._widget.restrict_ns.isChecked()) and (self._widget.namespace_input.currentText() in path)) or (path == self._widget.namespace_input.currentText()[0:-1])):
            self._widget.path_input.addItem(path)
            self._widget.ud_path_input.addItem(path)

    def _update_graph(self):
        """This thread continuously updates the graph when it changes.

        The graph gets updated in one of two ways:

          1: The structure of the SMACH plans has changed, or the display
          settings have been changed. In this case, the dotcode needs to be
          regenerated.

          2: The status of the SMACH plans has changed. In this case, we only
          need to change the styles of the graph.
        """
        if self._keep_running and self._graph_needs_refresh and not rospy.is_shutdown():
                # Get the containers to update
            containers_to_update = {}
            # Check if the path that's currently being viewed is in the
            # list of existing containers
            if self._path in self._containers:
                # Some non-root path
                containers_to_update = {self._path:self._containers[self._path]}
            elif self._path == '/':
                # Root path
                containers_to_update = self._top_containers

            # Check if we need to re-generate the dotcode (if the structure changed)
            # TODO: needs_zoom is a misnomer
            if self._structure_changed or self._needs_zoom or self._graph_needs_refresh:
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
                # TODO: Only re-generate dotcode for containers that have changed
                for path,container in containers_to_update.items():
                    dotstr += container.get_dotcode(
                            self._selected_paths,[],
                            0,self._max_depth,
                            self._containers,
                            self._show_all_transitions,
                            self._label_wrapper)

                # The given path isn't available
                if len(containers_to_update) == 0:
                    dotstr += '"__empty__" [label="Path not available.", shape="plaintext"]'

                dotstr += '\n}\n'

                # Set the dotcode to the new dotcode, reset the flags
                self.set_dotcode(dotstr,zoom=False)
                self._structure_changed = False
                self._graph_needs_refresh = False

            # Update the styles for the graph if there are any updates
            for path,container in containers_to_update.items():
                container.set_styles(
                        self._selected_paths,
                        0,self._max_depth,
                        self._widget.xdot_widget.items_by_url,
                        self._widget.xdot_widget.subgraph_shapes,
                        self._containers)
            #self.set_dotcode(dotstr)

    def set_dotcode(self, dotcode, zoom=True):
        """Set the xdot view's dotcode and refresh the display."""
        # Set the new dotcode
        if self._widget.xdot_widget.set_dotcode(dotcode, False):
            # Re-zoom if necessary
            if zoom or self._needs_zoom:
                self._widget.xdot_widget.zoom_to_fit()
                self._needs_zoom = False
            # Set the refresh flag
            #self._needs_refresh = True

    def _update_tree(self):
        """Update the tree view."""
        if self._keep_running and self._tree_needs_refresh and not rospy.is_shutdown():
            self._tree_nodes = {}
            self._widget.tree.clear()
            for path,tc in self._top_containers.iteritems():
                if ((not self._widget.restrict_ns.isChecked()) or ((self._widget.restrict_ns.isChecked()) and (self._widget.namespace_input.currentText() in path)) or (path == self._widget.namespace_input.currentText()[0:-1])):
                    self.add_to_tree(path, None)
            self._tree_needs_refresh = False
            self._widget.tree.sortItems(0,0)

    def add_to_tree(self, path, parent):
        """Add a path to the tree view."""
        if parent is None:
            container = QTreeWidgetItem()
            container.setText(0, self._containers[path]._label)
            self._widget.tree.addTopLevelItem(container)
        else:
            container = QTreeWidgetItem(parent)
            container.setText(0, self._containers[path]._label)

        # Add children to_tree
        for label in self._containers[path]._children:
            child_path = '/'.join([path,label])
            if child_path in self._containers.keys():
                self.add_to_tree(child_path, container)
            else:
                child = QTreeWidgetItem(container)
                child.setText(0, label)

    def append_tree(self, container, parent = None):
        """Append an item to the tree view."""
        if not parent:
            node = QTreeWidgetItem()
            node.setText(0, container._label)
            self._widget.tree.addTopLevelItem(node)
            for child_label in container._children:
                child = QTreeWidgetItem(node)
                child.setText(0, child_label)

    def _update_thread_run(self):
        """Update the list of namespaces."""
        _, _, topic_types = rospy.get_master().getTopicTypes()
        self._topic_dict = dict(topic_types)
        keys = list(self._topic_dict.keys())
        namespaces = list()
        for i in keys:
            print i
            if i.endswith("smach/container_status"):
                namespaces.append(i[0:i.index("smach/container_status")])
        self._widget.namespace_input.setItems.emit(namespaces)

    @Slot()
    def _update_finished(self):
        """Enable namespace combo box."""
        self._widget.namespace_input.setEnabled(True)

    def _handle_ns_changed(self):
        """If namespace selection is changed then reinitialize everything."""
        ns = self._widget.namespace_input.currentText()
        if len(ns) > 0:
            if self._ns != ns:
                self._actions_connected = False
                self._ns = ns
                self.enable_widgets(False)
                rospy.loginfo("Creating action clients on namespace '%s'..." % ns)
                rospy.loginfo("Action clients created.")
                self._actions_connected = True
                self.enable_widgets(True)

                self._containers = {}
                self._top_containers = {}
                self._selected_paths = []

                self._structure_subs = {}
                self._status_subs = {}

                self._needs_zoom = True
                self._structure_changed = True
                self._max_depth = -1
                self._show_all_transitions = False
                self._graph_needs_refresh = True
                self._tree_needs_refresh = True

                #self._widget.namespace_input.clear()
                self._widget.path_input.clear()
                self._widget.ud_path_input.clear()
                self._widget.tree.clear()

    @Slot()
    def refresh_combo_box(self):
        """Refresh namespace combo box."""
        self._update_thread.kill()
        self._containers = {}
        self._top_containers = {}
        self._selected_paths = []

        self._structure_subs = {}
        self._status_subs = {}

        self._needs_zoom = True
        self._structure_changed = True
        self._max_depth = -1
        self._show_all_transitions = False
        self._graph_needs_refresh = True
        self._tree_needs_refresh = True

        #self._widget.namespace_input.clear()
        self._widget.path_input.clear()
        self._widget.ud_path_input.clear()
        self._widget.tree.clear()
        if self._widget.restrict_ns.isChecked():
            self._widget.namespace_input.setEnabled(False)
            self._widget.namespace_input.setEditText('updating...')
            self._widget.ns_refresh_button.setEnabled(True)
            self._update_thread.start()
        else:
            self._widget.namespace_input.setEnabled(False)
            self._widget.namespace_input.setEditText('Unrestricted')
            self._widget.ns_refresh_button.setEnabled(False)
            self._graph_needs_refresh = True
            self._tree_needs_refresh = True
            self._widget.path_input.addItem('/')

    def _handle_path_changed(self, checked):
        """If the path input is changed, update graph."""
        self._path = self._widget.path_input.currentText()
        self._graph_needs_refresh = True
        self._needs_zoom = True

    def enable_widgets(self, enable):
        """Enable all widgets."""
        self._widget.xdot_widget.setEnabled(enable)
        self._widget.path_input.setEnabled(enable)
        self._widget.depth_input.setEnabled(enable)
        self._widget.label_width_input.setEnabled(enable)
        self._widget.ud_path_input.setEnabled(enable)
        self._widget.ud_text_browser.setEnabled(enable)
