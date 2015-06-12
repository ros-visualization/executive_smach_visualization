import os
import rospy
import roslib
import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtGui import QStyle,QApplication,QMouseEvent
from python_qt_binding.QtCore import * #Qt
from python_qt_binding.QtCore import Signal,Slot
from python_qt_binding.QtGui import QWidget,QPalette,QColor,QStandardItemModel,QItemDelegate,QStyleOptionButton,QStandardItem,QIcon
from python_qt_binding.QtCore import Qt,QTimer,Signal,QRect,QSize,QEvent
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

from PySide import QtGui

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

        #self._widget.subscribe_button.setCheckable(True)
        #TODO do for both combo boxes
        self._widget.namespace_input.currentIndexChanged.connect(self._handle_refresh_clicked)
        self._widget.ns_refresh_button.clicked.connect(self.refresh_combo_box)
        self._widget.restrict_ns.stateChanged.connect(self.refresh_combo_box)

        self._widget.tree.setColumnCount(1)
        self._widget.tree.setHeaderLabels(['Containers'])
        self._qlist = [] #QStringList()

        self._ns = ""
        self.refresh_combo_box()

        # Bind path list
        self._widget.path_input.currentIndexChanged.connect(
                self._handle_path_changed)

        #OLD v. NEW (RQT -> smach)

        # Create graph data structures
        # Containers is a map of (container path) -> container proxy
        self._containers = {}
        self._top_containers = {}

        # This is triggered every time the display needs to be updated
        #TODO Qt equivalent
        self._update_cond = threading.Condition()

        # smach introspection client
        self._client = smach_ros.IntrospectionClient()
        self._containers= {}
        self._selected_paths = []

        # Message subscribers
        self._structure_subs = {}
        self._status_subs = {}

        # Populate the frame with widgets
        #self._populate_frame()

        #TODO Register mouse event callback for xdot widget
        #self.widget.register_select_callback(self.select_cb)

        # Initialize xdot display state
        self._path = '/'
        self._widget.path_input.addItem('/')
        self._needs_zoom = True
        self._structure_changed = True
        self._needs_refresh = True
        self._max_depth = -1
        self._show_all_transitions = False
        self._label_wrapper = textwrap.TextWrapper(40,break_long_words=True)


        # Start a thread in the background to update the server list
        self._keep_running = True
        #self._server_list_thread = threading.Thread(target=self._update_server_list)
        #self._server_list_thread.start()
        self._server_list_thread = WorkerThread(self._update_server_list)
        self._server_list_thread.start()

        # Start a thread in the background to update the graph display
        #self._update_graph_thread = threading.Thread(target=self._update_graph)
        #self._update_graph_thread.start()
        self._update_graph_thread = WorkerThread(self._update_graph)
        self._update_graph_thread.start()
        # Start a thread in the background to update the._widget.tree display
        #self._update._widget.tree_thread = threading.Thread(target=self._update._widget.tree)
        #self._update._widget.tree_thread.start()
        self._update_tree_thread = WorkerThread(self._update_tree)
        self._update_tree_thread.start()

    def _update_server_list(self):
        """Update the list of known SMACH introspection servers."""
        while self._keep_running:
            # Discover new servers
            #TODO limit to namespace in top bar
            #if not self._widget.restrict_ns.isChecked():
            server_names = self._client.get_servers()
            new_server_names = [sn for sn in server_names if sn not in self._status_subs]
            #else:
            #    new_server_names = [self._widget.namespace_input.currentText()[0:(-1)]]
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

            # We need to redraw thhe graph if this container's parent is already known
            if parent_path in self._containers:
                needs_redraw = True

        # Update the graph if necessary
        if needs_redraw:
            with self._update_cond:
                self._structure_changed = True
                self._needs_zoom = True # TODO: Make it so you can disable this
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
            # Get the container and check if the status update requires regeneration
            container = self._containers[path]
            if container.update_status(msg):
                with self._update_cond:
                    self._update_cond.notify_all()

            # TODO: Is this necessary?
            """path_input_str = self.path_input.GetValue()
            if path_input_str == path or get_parent_path(path_input_str) == path:
                wx.PostEvent(
                        self.path_input.GetEventHandler(),
                        wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED,self.path_input.GetId()))
            """

    def _append_new_path(self, path):
        """Append a new path to the path selection widgets"""
        self._widget.path_input.addItem(path)
        self._widget.ud_path_input.addItem(path)
        return

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
                # Wait for the update condition to be triggered
                self._update_cond.wait()

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

                # Update the styles for the graph if there are any updates
                #TODO
                """for path,container in containers_to_update.items():
                    container.set_styles(
                            self._selected_paths,
                            0,self._max_depth,
                            self.widget.items_by_url,
                            self.widget.subgraph_shapes,
                            self._containers)
                """
                # Redraw xdot widget
                #TODO force OnIdle
                #self.widget.Refresh()

    def set_dotcode(self, dotcode, zoom=True):
        """Set the xdot view's dotcode and refresh the display."""
        # Set the new dotcode
        if self._widget.xdot_widget.set_dotcode(dotcode, False):
            # Re-zoom if necessary
            if zoom or self._needs_zoom:
                self._widget.xdot_widget.zoom_to_fit()
                self._needs_zoom = False
            # Set the refresh flag
            self._needs_refresh = True
            #wx.PostEvent(self.GetEventHandler(), wx.IdleEvent())


    def _update_tree(self):
        """Update the tree view."""
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                self._update_cond.wait()
                #TODO replace with qt delete._widget.tree
                self._widget.tree.clear()
                self._tree_nodes = {}
                for path,tc in self._top_containers.iteritems():
                    self.add_to_tree(path, None)
                self._widget.tree.show()

    def add_to_tree(self, path, parent):
        """Add a path to the tree view."""
        #TODO
        if parent is None:
            self._qlist = [get_label(path)]
            container = QtGui.QTreeWidgetItem(self._widget.tree, self._qlist)
            #self._widget.tree.addTopLevelItem(get_label(path))
        else:
            self._qlist = [get_label(path)]
            container = QtGui.QTreeWidgetItem(parent, self._qlist)
            #self._widget.tree.AppendItem(parent,get_label(path))

        # Add children to_tree
        for label in self._containers[path]._children:
            child_path = '/'.join([path,label])
            if child_path in self._containers.keys():
                self.add_to_tree(child_path, container)
            else:
                self._qlist = [label]
                QtGui.QTreeWidgetItem(container, self._qlist)
                #self._widget.tree.AppendItem(container,label)


    def append_tree(self, container, parent = None):
        """Append an item to the tree view."""
        #TODO
        if not parent:
            self._qlist = [container._label]
            node = QtGui.QTreeWidgetItem(self._widget.tree, self._qlist)
            #self._widget.tree.addTopLevelItem(container._label)
            for child_label in container._children:
                self._qlist = [child_label]
                QtGui.QTreeWidgetItem(node, self._qlist)
                #self._widget.tree.AppendItem(node,child_label)


    def _update_thread_run(self):
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
        self._widget.namespace_input.setEnabled(True)

    def _handle_refresh_clicked(self, checked):
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

    @Slot()
    def refresh_combo_box(self):
        self._update_thread.kill()
        if self._widget.restrict_ns.isChecked():
            self._widget.namespace_input.setEnabled(False)
            self._widget.namespace_input.setEditText('updating...')
            self._update_thread.start()
        else:
            self._widget.namespace_input.setEnabled(False)
            self._widget.namespace_input.setEditText('Unrestricted')

    def _handle_path_changed(self, checked):
        self._path = self._widget.path_input.currentText()
        with self._update_cond:
            self._update_cond.notifyAll()
        #TODO trigger thread?

    def enable_widgets(self, enable):
        self._widget.xdot_widget.setEnabled(enable)
        self._widget.path_input.setEnabled(enable)
        self._widget.depth_input.setEnabled(enable)
        self._widget.label_width_input.setEnabled(enable)
        self._widget.ud_path_input.setEnabled(enable)
        self._widget.ud_text_browser.setEnabled(enable)
