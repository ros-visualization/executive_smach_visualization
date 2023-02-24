#!/usr/bin/env python

# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
# Copyright (c) 2013, Jonathan Bohren, The Johns Hopkins University
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#   * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#   * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jonathan Bohren

import rospkg
import rospy

from sensor_msgs.msg import Image

import cv_bridge
import os
import sys
import threading
import time
import numpy as np

from distutils.version import LooseVersion

try:
    import wxversion
    if wxversion.checkInstalled("2.8"):
        wxversion.select("2.8")
    else:
        print("wxversion 2.8 is not installed, installed versions are {}".format(wxversion.getInstalled()))

    ## this import system (or ros-released) xdot
    # import xdot
    ## need to import currnt package, but not to load this file
    # http://stackoverflow.com/questions/6031584/importing-from-builtin-library-when-module-with-same-name-exists
    def import_non_local(name, custom_name=None):
        import imp, sys

        custom_name = custom_name or name

        path = filter(lambda x: x != os.path.dirname(os.path.abspath(__file__)), sys.path)
        f, pathname, desc = imp.find_module(name, path)

        module = imp.load_module(custom_name, f, pathname, desc)
        if f:
            f.close()

        return module

    smach_viewer = import_non_local('smach_viewer')
    from smach_viewer import xdot
    from smach_viewer.xdot import wxxdot
    from smach_viewer.xdot.xdot import TextShape
except:
    # Guard against self import
    this_dir = os.path.dirname(__file__)
    # Use os.getcwd() to aovid weird symbolic link problems
    cur_dir = os.getcwd()
    os.chdir(this_dir)
    this_dir_cwd = os.getcwd()
    os.chdir(cur_dir)
    # Remove this dir from path
    sys.path = [a for a in sys.path if a not in [this_dir, this_dir_cwd]]
    # Ignore path ending with smach_viewer/lib/smach_viewer
    sys.path = [a for a in sys.path if not a.endswith('smach_viewer/lib/smach_viewer')]
    #
    from smach_viewer.xdot import wxxdot
    from xdot.ui.elements import *


import wx
import wx.richtext

from smach_viewer.smach_viewer_base import ContainerNode
from smach_viewer.smach_viewer_base import SmachViewerBase
from smach_viewer.utils import get_label
from smach_viewer.utils import get_parent_path
from smach_viewer.utils import hex2t


class WxContainerNode(ContainerNode):
    def set_styles(self, selected_paths, depth, max_depth, items, subgraph_shapes, containers):
        """Update the styles for a list of containers without regenerating the dotcode.

        This function is called recursively to update an entire tree.
        
        @param selected_paths: A list of paths to nodes that are currently selected.
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to traverse into the tree
        @param items: A dict of all the graph items, keyed by url
        @param subgraph_shapes: A dictionary of shapes from the rendering engine
        @param containers: A dict of all the containers
        """

        # Color root container
        """
        if depth == 0:
            container_shapes = subgraph_shapes['cluster_'+self._path]
            container_color = (0,0,0,0)
            container_fillcolor = (0,0,0,0)

            for shape in container_shapes:
                shape.pen.color = container_color
                shape.pen.fillcolor = container_fillcolor
                """

        # Color shapes for outcomes

        # Color children
        if max_depth == -1 or depth <= max_depth:
            # Iterate over children
            for child_label in self._children:
                child_path = '/'.join([self._path,child_label])

                child_color = [0.5,0.5,0.5,1]
                child_fillcolor = [1,1,1,1]
                child_linewidth = 2

                active_color = hex2t('#5C7600FF')
                active_fillcolor = hex2t('#C0F700FF')

                initial_color = hex2t('#000000FF')

                if child_label in self._active_states:
                    # Check if the child is active
                    child_color = active_color
                    child_fillcolor = active_fillcolor
                    child_linewidth = 5
                elif child_label in self._initial_states:
                    # Initial style
                    #child_fillcolor = initial_fillcolor
                    child_color = initial_color
                    child_linewidth = 2

                # Check if the child is selected
                if child_path in selected_paths:
                    child_color = hex2t('#FB000DFF')

                # Generate dotcode for child containers 
                if child_path in containers:
                    subgraph_id = 'cluster_'+child_path
                    if subgraph_id in subgraph_shapes:
                        if child_label in self._active_states:
                            child_fillcolor[3] = 0.25
                        elif 0 and child_label in self._initial_states:
                            child_fillcolor[3] = 0.25
                        else:
                            if max_depth > 0:
                                v = 1.0-0.25*((depth+1)/float(max_depth))
                            else:
                                v = 0.85
                            child_fillcolor = [v,v,v,1.0]

                        for shape in subgraph_shapes['cluster_'+child_path]:
                            pen = shape.pen
                            if len(pen.color) > 3:
                                pen_color_opacity = pen.color[3]
                                if pen_color_opacity < 0.01:
                                    pen_color_opacity = 0
                            else:
                                pen_color_opacity = 0.5
                            shape.pen.color = child_color[0:3]+[pen_color_opacity]
                            shape.pen.fillcolor = [child_fillcolor[i] for i in range(min(3,len(pen.fillcolor)))]
                            shape.pen.linewidth = child_linewidth

                        # Recurse on this child
                        containers[child_path].set_styles(
                                selected_paths,
                                depth+1, max_depth,
                                items,
                                subgraph_shapes,
                                containers)
                else:
                    if child_path in items:
                        for shape in items[child_path].shapes:
                            if not isinstance(shape, TextShape):
                                shape.pen.color = child_color
                                shape.pen.fillcolor = child_fillcolor
                                shape.pen.linewidth = child_linewidth
                    else:
                        # print child_path+" NOT IN "+str(items.keys())
                        pass


class SmachViewerFrame(wx.Frame, SmachViewerBase):
    """
    This class provides a GUI application for viewing SMACH plans.
    """

    _container_class = WxContainerNode

    def __init__(self):
        wx.Frame.__init__(self, None, -1, "Smach Viewer", size=(720,480))
        SmachViewerBase.__init__(self)
        vbox = wx.BoxSizer(wx.VERTICAL)

        # Create Splitter
        self.content_splitter = wx.SplitterWindow(self, -1,style = wx.SP_LIVE_UPDATE)
        self.content_splitter.SetMinimumPaneSize(24)
        self.content_splitter.SetSashGravity(0.85)

        # Create viewer pane
        viewer = wx.Panel(self.content_splitter,-1)

        # Create smach viewer 
        nb = wx.Notebook(viewer,-1,style=wx.NB_TOP | wx.WANTS_CHARS)
        viewer_box = wx.BoxSizer()
        viewer_box.Add(nb,1,wx.EXPAND | wx.ALL, 4)
        viewer.SetSizer(viewer_box)

        # Create graph view
        graph_view = wx.Panel(nb,-1)
        gv_vbox = wx.BoxSizer(wx.VERTICAL)
        graph_view.SetSizer(gv_vbox)

        # Construct toolbar
        toolbar = wx.ToolBar(graph_view, -1)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"Path: "))

        # Path list
        self.path_combo = wx.ComboBox(toolbar, -1, style=wx.CB_DROPDOWN)
        self.path_combo .Bind(wx.EVT_COMBOBOX, self.set_path)
        self.path_combo.Append('/')
        self.path_combo.SetValue('/')
        toolbar.AddControl(self.path_combo)

        # Depth spinner
        self.depth_spinner = wx.SpinCtrl(toolbar, -1,
                size=wx.Size(50,-1),
                min=-1,
                max=1337,
                initial=-1)
        self.depth_spinner.Bind(wx.EVT_SPINCTRL,self.set_depth)
        toolbar.AddControl(wx.StaticText(toolbar,-1,"    Depth: "))
        toolbar.AddControl(self.depth_spinner)

        # Label width spinner
        self.width_spinner = wx.SpinCtrl(toolbar, -1,
                size=wx.Size(50,-1),
                min=1,
                max=1337,
                initial=40)
        self.width_spinner.Bind(wx.EVT_SPINCTRL,self.set_label_width)
        toolbar.AddControl(wx.StaticText(toolbar,-1,"    Label Width: "))
        toolbar.AddControl(self.width_spinner)

        # Implicit transition display
        toggle_all = wx.ToggleButton(toolbar,-1,'Show Implicit')
        toggle_all.Bind(wx.EVT_TOGGLEBUTTON, self.toggle_all_transitions)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"    "))
        toolbar.AddControl(toggle_all)

        toggle_auto_focus = wx.ToggleButton(toolbar, -1, 'Auto Focus')
        toggle_auto_focus.Bind(wx.EVT_TOGGLEBUTTON, self.toggle_auto_focus)
        self._auto_focus = False

        toolbar.AddControl(wx.StaticText(toolbar, -1, "    "))
        toolbar.AddControl(toggle_auto_focus)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"    "))
        if LooseVersion(wx.__version__) >= LooseVersion('4.0'):
            toolbar.AddTool(wx.ID_HELP, 'Help',
                            wx.ArtProvider.GetBitmap(wx.ART_HELP,wx.ART_OTHER,(16,16)) )
            toolbar.AddTool(wx.ID_SAVE, 'Save',
                            wx.ArtProvider.GetBitmap(wx.ART_FILE_SAVE,wx.ART_OTHER,(16,16)) )
        else:
            toolbar.AddLabelTool(wx.ID_HELP, 'Help',
                                 wx.ArtProvider.GetBitmap(wx.ART_HELP,wx.ART_OTHER,(16,16)) )
            toolbar.AddLabelTool(wx.ID_SAVE, 'Save',
                                 wx.ArtProvider.GetBitmap(wx.ART_FILE_SAVE,wx.ART_OTHER,(16,16)) )
        toolbar.Realize()

        self.Bind(wx.EVT_TOOL, self.ShowControlsDialog, id=wx.ID_HELP)
        self.Bind(wx.EVT_TOOL, self.SaveDotGraph, id=wx.ID_SAVE)

        # Create dot graph widget
        self.widget = wxxdot.WxDotWindow(graph_view, -1)

        gv_vbox.Add(toolbar, 0, wx.EXPAND)
        gv_vbox.Add(self.widget, 1, wx.EXPAND)

        # Create tree view widget
        self.tree = wx.TreeCtrl(nb,-1,style=wx.TR_HAS_BUTTONS)
        nb.AddPage(graph_view,"Graph View")
        nb.AddPage(self.tree,"Tree View")


        # Create userdata widget
        borders = wx.LEFT | wx.RIGHT | wx.TOP
        border = 4
        self.ud_win = wx.ScrolledWindow(self.content_splitter, -1)
        self.ud_gs = wx.BoxSizer(wx.VERTICAL)

        self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Path:"),0, borders, border)

        self.path_input = wx.ComboBox(self.ud_win,-1,style=wx.CB_DROPDOWN)
        self.path_input.Bind(wx.EVT_COMBOBOX,self.selection_changed)
        self.ud_gs.Add(self.path_input,0,wx.EXPAND | borders, border)


        self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Userdata:"),0, borders, border)

        self.ud_txt = wx.TextCtrl(self.ud_win,-1,style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.ud_gs.Add(self.ud_txt,1,wx.EXPAND | borders, border)
        
        # Add initial state button
        self.is_button = wx.Button(self.ud_win,-1,"Set as Initial State")
        self.is_button.Bind(wx.EVT_BUTTON, self.on_set_initial_state)
        self.is_button.Disable()
        self.ud_gs.Add(self.is_button,0,wx.EXPAND | wx.BOTTOM | borders, border)

        self.ud_win.SetSizer(self.ud_gs)


        # Set content splitter
        self.content_splitter.SplitVertically(viewer, self.ud_win, 512)

        # Add statusbar
        self.statusbar = wx.StatusBar(self,-1)

        # Add elements to sizer
        vbox.Add(self.content_splitter, 1, wx.EXPAND | wx.ALL)
        vbox.Add(self.statusbar, 0, wx.EXPAND)

        self.SetSizer(vbox)
        self.Center()

        self._pub = rospy.Publisher('~image', Image, queue_size=1)

        self.Bind(wx.EVT_IDLE,self.OnIdle)
        self.Bind(wx.EVT_CLOSE,self.OnQuit)

        # Register mouse event callback
        self.widget.register_select_callback(self.select_cb)

        # Start a thread in the background to update the server list
        self._update_tree_thread = threading.Thread(target=self._update_tree)
        self._update_tree_thread.start()

        # image publish timer
        self.timer = wx.Timer(self, 0)
        self.Bind(wx.EVT_TIMER, self.OnTimer)
        self.timer.Start(200)


    def OnQuit(self,event):
        """Quit Event: kill threads and wait for join."""
        self.kill()
        self._update_tree_thread.join()
        event.Skip()

    def update_graph(self):
        """Notify all that the graph needs to be updated."""
        with self._update_cond:
            self._update_cond.notify_all()

    def on_set_initial_state(self, event):
        """Event: Change the initial state of the server."""
        state_path = self._selected_paths[0]
        parent_path = get_parent_path(state_path)
        state = get_label(state_path)

        server_name = self._containers[parent_path]._server_name
        self._client.set_initial_state(server_name,parent_path,[state],timeout = rospy.Duration(60.0))

    def set_path(self, event):
        """Event: Change the viewable path and update the graph."""
        self._path = self.path_combo.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def _set_path(self, path):
        self._path = path
        self._needs_zoom = True
        self.path_combo.SetValue(path)
        self.update_graph()

    def set_depth(self, event):
        """Event: Change the maximum depth and update the graph."""
        self._max_depth = self.depth_spinner.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def _set_max_depth(self, max_depth):
        self._max_depth = max_depth
        self.depth_spinner.SetValue(max_depth)
        self._needs_zoom = True
        self.update_graph()

    def set_label_width(self, event):
        """Event: Change the label wrapper width and update the graph."""
        self._label_wrapper.width = self.width_spinner.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def toggle_all_transitions(self, event):
        """Event: Change whether automatic transitions are hidden and update the graph."""
        self._show_all_transitions = not self._show_all_transitions
        self._structure_changed = True
        self.update_graph()

    def toggle_auto_focus(self, event):
        """Event: Enable/Disable automatically focusing"""
        self._auto_focus = not self._auto_focus
        self._needs_zoom = self._auto_focus
        self._structure_changed = True
        if not self._auto_focus:
            self._set_path('/')
            self._set_max_depth(-1)
        self.update_graph()

    def select_cb(self, item, event):
        """Event: Click to select a graph node to display user data and update the graph."""

        # Only set string status
        try:
            if not type(item.url) is str:
                return
        except AttributeError:
            return

        self.statusbar.SetStatusText(item.url)
        # Left button-up
        if event.ButtonUp(wx.MOUSE_BTN_LEFT):
            # Store this item's url as the selected path
            self._selected_paths = [item.url]
            # Update the selection dropdown
            self.path_input.SetValue(item.url)
            wx.PostEvent(
                    self.path_input.GetEventHandler(),
                    wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED,self.path_input.GetId()))
            self.update_graph()

    def selection_changed(self, event):
        """Event: Selection dropdown changed."""
        path_input_str = self.path_input.GetValue()

        # Check the path is non-zero length
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
                # Enable the initial state button for the selection
                self.is_button.Enable()

                # Get the container
                container = self._containers[parent_path]

                # Store the scroll position and selection
                pos = self.ud_txt.HitTestPos(wx.Point(0,0))
                sel = self.ud_txt.GetSelection()

                # Generate the userdata string
                ud_str = ''
                for (k,v) in container._local_data._data.items():
                    ud_str += str(k)+": "
                    vstr = str(v)
                    # Add a line break if this is a multiline value
                    if vstr.find('\n') != -1:
                        ud_str += '\n'
                    ud_str+=vstr+'\n\n'

                # Set the userdata string
                self.ud_txt.SetValue(ud_str)

                # Restore the scroll position and selection
                self.ud_txt.ShowPosition(pos[1])
                if sel != (0,0):
                    self.ud_txt.SetSelection(sel[0],sel[1])
            else:
                # Disable the initial state button for this selection
                self.is_button.Disable()

    def _structure_msg_update(self, msg, server_name):
        """Update the structure of the SMACH plan (re-generate the dotcode)."""

        # Just return if we're shutting down
        if not self._keep_running:
            return

        SmachViewerBase._structure_msg_update(self, msg, server_name)
        path = msg.path
        if path not in self._containers:
            self.path_combo.Append(path)
            self.path_input.Append(path)

    def _status_msg_update(self, msg):
        """Process status messages."""

        # Check if we're in the process of shutting down
        if not self._keep_running:
            return

        if self._auto_focus and len(msg.info) > 0:
            self._set_path(msg.info)
            self._set_max_depth(msg.info.count('/')-1)

        SmachViewerBase._status_msg_update(self, msg)
        path = msg.path
        # Check if this is a known container
        if path not in self._containers:
            return

        # TODO(???): Is this necessary?
        path_input_str = self.path_input.GetValue()
        if path_input_str == path or get_parent_path(path_input_str) == path:
            wx.PostEvent(
                    self.path_input.GetEventHandler(),
                    wx.CommandEvent(
                        wx.wxEVT_COMMAND_COMBOBOX_SELECTED,
                        self.path_input.GetId()))

    def _update_graph_step(self):
        containers_to_update = SmachViewerBase._update_graph_step(self)

        # Get the containers to update
        if self._structure_changed or self._needs_zoom:
            # Set the dotcode to the new dotcode, reset the flags
            try:
                self.set_dotcode(self.dotstr, zoom=False)
            except UnicodeDecodeError as e:
                # multibyte language only accepts even number
                label_width = self._label_wrapper.width
                rospy.logerr('label width {} causes error'.format(label_width))
                rospy.logerr('maybe multibyte word is in your label.')
                rospy.logerr(e)
                rospy.logerr('changing width label width to {}'.format(
                    label_width + 1))
                self._label_wrapper.width = label_width + 1
            self._structure_changed = False

        if hasattr(self.widget, 'subgraph_shapes'):
            # Update the styles for the graph if there are any updates
            for path, tc in containers_to_update.items():
                tc.set_styles(
                        self._selected_paths,
                        0, self._max_depth,
                        self.widget.items_by_url,
                        self.widget.subgraph_shapes,
                        self._containers)

        # Redraw
        self.widget.Refresh()

    def set_dotcode(self, dotcode, zoom=True):
        """Set the xdot view's dotcode and refresh the display."""
        # Set the new dotcode
        if self.widget.set_dotcode(dotcode, None):
            self.SetTitle('Smach Viewer')
            # Re-zoom if necessary
            if zoom or self._needs_zoom:
                self.widget.zoom_to_fit()
                self._needs_zoom = False
            # Set the refresh flag
            self._needs_refresh = True
            wx.PostEvent(self.GetEventHandler(), wx.IdleEvent())

    def _update_tree(self):
        """Update the tree view."""
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                self._update_cond.wait()
                self.tree.DeleteAllItems()
                self._tree_nodes = {}
                for path,tc in self._top_containers.items():
                    self.add_to_tree(path, None)

    def add_to_tree(self, path, parent):
        """Add a path to the tree view."""
        if parent is None:
            container = self.tree.AddRoot(get_label(path))
        else:
            container = self.tree.AppendItem(parent,get_label(path))

        # Add children to tree
        for label in self._containers[path]._children:
            child_path = '/'.join([path,label])
            if child_path in list(self._containers.keys()):
                self.add_to_tree(child_path, container)
            else:
                self.tree.AppendItem(container,label)

    def append_tree(self, container, parent = None):
        """Append an item to the tree view."""
        if not parent:
            node = self.tree.AddRoot(container._label)
            for child_label in container._children:
                self.tree.AppendItem(node,child_label)

    def OnIdle(self, event):
        """Event: On Idle, refresh the display if necessary, then un-set the flag."""
        if self._needs_refresh:
            self.Refresh()
            # Re-populate path combo
            self._needs_refresh = False

    def OnTimer(self, event):
        if self._pub.get_num_connections() < 1:
            rospy.logwarn_once("Publishing {} requires at least one subscriber".format(self._pub.name))
            return
        # image
        context = wx.ClientDC(self)
        memory = wx.MemoryDC()
        x, y = self.ClientSize
        if LooseVersion(wx.__version__) >= LooseVersion('4.0'):
            bitmap = wx.Bitmap(x, y, -1)
        else:
            bitmap = wx.EmptyBitmap(x, y, -1)
        memory.SelectObject(bitmap)
        memory.Blit(0, 0, x, y, context, 0, 0)
        memory.SelectObject(wx.NullBitmap)
        if LooseVersion(wx.__version__) >= LooseVersion('4.0'):
            buf = bitmap.ConvertToImage().GetDataBuffer()
        else:
            buf = wx.ImageFromBitmap(bitmap).GetDataBuffer()
        img = np.frombuffer(buf, dtype=np.uint8)
        bridge = cv_bridge.CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img.reshape((y, x, 3)), encoding='rgb8')
        img_msg.header.stamp = rospy.Time.now()
        self._pub.publish(img_msg)

    def ShowControlsDialog(self,event):
        dial = wx.MessageDialog(None,
                "Pan: Arrow Keys\nZoom: PageUp / PageDown\nZoom To Fit: F\nRefresh: R",
                'Keyboard Controls', wx.OK)
        dial.ShowModal()

    def SaveDotGraph(self,event):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        directory = rospkg.get_ros_home()+'/dotfiles/'
        if not os.path.exists(directory):
                os.makedirs(directory)
        filename = directory+timestr+'.dot'
        print('Writing to file: %s' % filename)
        with open(filename, 'w') as f:
            f.write(self.dotstr)

    def OnExit(self, event):
        pass

    def set_filter(self, filter):
        self.widget.set_filter(filter)

def main():
    from argparse import ArgumentParser
    p = ArgumentParser()
    p.add_argument('-f', '--auto-focus',
                 action='store_true',
                 help="Enable 'AutoFocus to subgraph' as default",
                 dest='enable_auto_focus')
    args = p.parse_args()
    app = wx.App()

    frame = SmachViewerFrame()
    frame.set_filter('dot')

    frame.Show()

    if args.enable_auto_focus:
        frame.toggle_auto_focus(None)

    app.MainLoop()

if __name__ == '__main__':
    rospy.init_node('smach_viewer',anonymous=False, disable_signals=True,log_level=rospy.INFO)
    sys.argv = rospy.myargv()
    main()
