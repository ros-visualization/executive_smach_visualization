import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtGui import QStyle,QApplication,QMouseEvent
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal,Slot
from python_qt_binding.QtGui import QWidget,QPalette,QColor,QStandardItemModel,QItemDelegate,QStyleOptionButton,QStandardItem,QIcon
from python_qt_binding.QtCore import Qt,QTimer,Signal,QRect,QSize,QEvent
from rqt_py_common.extended_combo_box import ExtendedComboBox
from qt_gui_py_common.worker_thread import WorkerThread

import threading

from xdot.xdot_qt import DotWidget

import std_msgs.msg

import actionlib

import smach_msgs.msg

from actionlib_msgs.msg import GoalStatus

from smach_msgs.msg import SmachContainerStatus,SmachContainerInitialStatusCmd,SmachContainerStructure

import .container_node

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
            self.conman._update_blocks()

        return success

class GroupsDelegate(QItemDelegate):
    def __init__(self, conman=None, parent=None):
        super(GroupsDelegate, self).__init__(parent)
        self.conman = conman

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
                    self.conman._enable_group(index.row(),True)
                elif index.column() == 2:
                    self.conman._enable_group(index.row(),False)


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
        self._widget.namespace_input.currentIndexChanged.connect(self._handle_refresh_clicked)
        self._widget.ns_refresh_button.clicked.connect(self.refresh_combo_box)

        self._ns = ""
        self.refresh_combo_box()

        # Create graph
        self._containers = {}
        self._top_containers = {}
        self._update_cond = threading.Condition()
        self._needs_refresh = True

        # Bind path list
        self._widget.path_input.currentIndexChanged.connect(
                self._handle_path_changed)

        # Message subscribers
        self._structure_subs = {}
        self._status_subs = {}

        return

        self._widget.refresh_button.clicked[bool].connect(self._handle_refresh_clicked)
        self._widget.commit_button.clicked[bool].connect(self._handle_commit_clicked)

        #self._widget.xdot_widget.connect(
                #self._widget.xdot_widget, SIGNAL('_update_graph'), self._widget.xdot_widget.set_dotcode)
        self.update_graph_sig.connect(self._update_graph)

        self.blocks = { }
        self.groups = { }

        self._actions_connected = False
        self.enable_widgets(False)
        self.new_dotcode_data = ''

        self.update_timer = QTimer(self)
        self.update_timer.setInterval(50)
        self.update_timer.timeout.connect(self._update_widgets)
        #self.update_timer.start()


        self._get_blocks = None
        self._set_blocks = None

        self._blocks_model = QStandardItemModel(0,4)
        self._blocks_model.setHeaderData(0, Qt.Horizontal, "")
        self._blocks_model.setHeaderData(1, Qt.Horizontal, "Action")
        self._blocks_model.setHeaderData(2, Qt.Horizontal, "State")
        self._blocks_model.setHeaderData(3, Qt.Horizontal, "Block")
        self._widget.blocks_table.setModel(self._blocks_model)
        self._blocks_delegate = BlocksDelegate(self)
        self._widget.blocks_table.setItemDelegate(self._blocks_delegate)
        self._blocks_model.itemChanged.connect(self.block_changed)

        self._groups_model = QStandardItemModel(0,4)
        self._groups_model.setHeaderData(0, Qt.Horizontal, "")
        self._groups_model.setHeaderData(1, Qt.Horizontal, "")
        self._groups_model.setHeaderData(2, Qt.Horizontal, "")
        self._groups_model.setHeaderData(3, Qt.Horizontal, "Group")
        self._widget.groups_table.setModel(self._groups_model)
        self._groups_delegate = GroupsDelegate(self)
        self._widget.groups_table.setItemDelegate(self._groups_delegate)


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

                #self._dotcode_sub = rospy.Subscriber(
                        #ns+'/dotcode',
                        #std_msgs.msg.String,
                        #self._dotcode_msg_cb)

                #self._get_blocks = actionlib.SimpleActionClient(
                        #ns+'/get_blocks_action',
                        #conman_msgs.msg.GetBlocksAction)

                #self._set_blocks = actionlib.SimpleActionClient(
                        #ns+'/set_blocks_action',
                        #conman_msgs.msg.SetBlocksAction)

                rospy.loginfo("Creating action clients on namespace '%s'..." % ns)

                #if not self._get_blocks.wait_for_server(rospy.Duration(2)):
                    #rospy.loginfo("Timed out waiting for %s." % self._get_blocks.action_client.ns)
                    #return
                #if not self._set_blocks.wait_for_server(rospy.Duration(2)):
                    #rospy.loginfo("Timed out waiting for %s." % self._set_blocks.action_client.ns)
                    #return

                rospy.loginfo("Action clients created.")
                self._actions_connected = True
                self.enable_widgets(True)

            #self._query_blocks()

    @Slot()
    def refresh_combo_box(self):
        self._update_thread.kill()
        self._widget.namespace_input.setEnabled(False)
        self._widget.namespace_input.setEditText('updating...')
        self._update_thread.start()

    def _handle_path_changed(self, checked):
        path = self._widget.path_input.currentText()


    def enable_widgets(self, enable):
        
        self._widget.xdot_widget.setEnabled(enable)
        self._widget.path_input.setEnabled(enable)
        self._widget.depth_input.setEnabled(enable)
        self._widget.label_width_input.setEnabled(enable)
        self._widget.ud_path_input.setEnabled(enable)
        self._widget.ud_text_browser.setEnabled(enable)


"""

    def block_changed(self, item):
        row = item.row()
        name = self._blocks_model.item(row,3).text()
        block = self.blocks[name]
        checked = item.checkState() == Qt.Checked

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._update_thread.kill()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def _get_result_cb(self, status, res):
        rospy.loginfo("got result!")
        self._blocks_model.setRowCount(0)
        self._blocks_model.setRowCount(len(res.blocks))

        for (i,block) in zip(range(len(res.blocks)),res.blocks):

            # Store in dict
            self.blocks[block.name] = block

            cb = QStandardItem(True)
            cb.setCheckable(True)
            cb.setCheckState(Qt.Checked if block.state.value == conman_msgs.msg.TaskState.RUNNING else Qt.Unchecked)
            cb.setTextAlignment(Qt.AlignHCenter)
            cb.setTristate(True)

            action = QStandardItem(True)
            action.setText("")

            state = QStandardItem(True)
            state.setText(state_map[block.state.value])

            name = QStandardItem(True)
            name.setText(str(block.name))

            self._blocks_model.setItem(i,0,cb)
            self._blocks_model.setItem(i,1,action)
            self._blocks_model.setItem(i,2,state)
            self._blocks_model.setItem(i,3,name)

        for (i,group) in zip(range(len(res.groups)),res.groups):

            self.groups[group.name] = group

            cb = QStandardItem(True)
            cb.setCheckable(True)
            cb.setCheckState(Qt.Checked)
            cb.setTextAlignment(Qt.AlignHCenter)
            cb.setEnabled(False)

            name = QStandardItem(True)
            name.setText(str(group.name))

            self._groups_model.setItem(i,0,cb)
            self._groups_model.setItem(i,3,name)

        self._update_groups()
        self._update_blocks()

        self._widget.blocks_table.resizeColumnsToContents()
        self._widget.blocks_table.horizontalHeader().setStretchLastSection(True)
        self._widget.groups_table.resizeColumnsToContents()
        self._widget.groups_table.horizontalHeader().setStretchLastSection(True)

    def _update_blocks(self):
        
        for (name, block) in self.blocks.items():
            items = self._blocks_model.findItems(name, column=3)
            if len(items) > 0:
                item = items[0]
            else:
                continue
            
            row = item.row()
            checked = self._blocks_model.item(row,0).checkState() == Qt.Checked
            if checked and block.state.value != conman_msgs.msg.TaskState.RUNNING:
                self._blocks_model.item(row,1).setText("ENABLE")
            elif not checked and block.state.value == conman_msgs.msg.TaskState.RUNNING:
                self._blocks_model.item(row,1).setText("DISABLE")
            else:
                self._blocks_model.item(row,1).setText("")
        self._update_groups()

    def _enable_group(self, index, enable):
        name = self._groups_model.item(index, 3).text()
        group = self.groups[name]

        for member in group.members:
            items = self._blocks_model.findItems(member, column=3)
            if len(items) > 0:
                item = items[0]
            else:
                continue
            
            row = item.row()
            self._blocks_model.item(row,0).setCheckState(Qt.Checked if enable else Qt.Unchecked)

        self._update_blocks()
        
    def _update_groups(self):
        for (name, group) in self.groups.items():
            group_items = self._groups_model.findItems(name, column=3)
            if len(group_items) > 0:
                group_item = group_items[0]
            else:
                continue
            group_row = group_item.row()

            members_checked = []

            for member in group.members:
                items = self._blocks_model.findItems(member, column=3)
                if len(items) > 0:
                    item = items[0]
                else:
                    continue
                row = item.row()
                members_checked.append(self._blocks_model.item(row,0).checkState() == Qt.Checked)

            if all(members_checked):
                check = Qt.Checked
            else:
                check = Qt.Unchecked

            self._groups_model.item(group_row,0).setCheckState(check)


    def _query_blocks(self):
        if self._get_blocks and self._actions_connected:
            if self._get_blocks.simple_state == actionlib.SimpleGoalState.DONE:
                rospy.loginfo("Getting blocks...")
                goal = conman_msgs.msg.GetBlocksGoal()
                goal.publish_flow_graph = self._widget.generate_graph_checkbox.isChecked()
                self._get_blocks.send_goal(goal, done_cb=self._get_result_cb)


    def _handle_commit_clicked(self, checked):
        if self._set_blocks and self._actions_connected:
            if self._get_blocks.simple_state == actionlib.SimpleGoalState.DONE:
                rospy.loginfo("Setting blocks...")
                goal = conman_msgs.msg.SetBlocksGoal()
                goal.diff = True
                goal.force = True

                for i in range(self._blocks_model.rowCount()):
                    name = self._blocks_model.item(i,3).text()
                    action = self._blocks_model.item(i,1).text()

                    if action == 'DISABLE':
                        goal.disable.append(name)
                    elif action == 'ENABLE':
                        goal.enable.append(name)

                self._set_blocks.send_goal(goal, done_cb=self._get_set_result_cb)

    def _get_set_result_cb(self, status, res):

        self._query_blocks()

    @Slot(str)
    def _update_graph(self,dotcode):
        if self.initialized: 
            self._widget.xdot_widget.set_dotcode(dotcode, center=False);
        else:
            self._widget.xdot_widget.set_dotcode(dotcode, center=True);
            self._widget.xdot_widget.zoom_to_fit()
            self.initialized = 1

    def _dotcode_msg_cb(self, msg):
        #self.new_dotcode_data = msg.data
        self.update_graph_sig.emit(msg.data)
    
    def _update_widgets(self):
        self._update_groups()
        self._update_blocks()
"""
