                        ui.markdown('Field settings').style('color: #6E93D6')
                        with ui.row():
                            self.with_field_planning = ui.checkbox('Use field planning', value=True) \
                                .bind_value(self.system.weeding, 'use_field_planning') \
                                .tooltip('Set the weeding automation to use the field planning with GNSS')

                            with ui.row().bind_visibility_from(self.with_field_planning, 'value', value=True):
                                self.show_start_row()
                                self.show_end_row()

                                ui.number('Min. turning radius', format='%.2f',
                                          value=0.5, step=0.05, min=0.05, max=2.0) \
                                    .props('dense outlined suffix=m').classes('w-30') \
                                    .bind_value(self.system.weeding, 'minimum_turning_radius') \
                                    .tooltip('Set the turning radius for the weeding automation')
                                ui.number('turn_offset', format='%.2f', value=0.4, step=0.05, min=0.05, max=2.0) \
                                    .props('dense outlined suffix=m').classes('w-30') \
                                    .bind_value(self.system.weeding, 'turn_offset') \
                                    .tooltip('Set the turning offset for the weeding automation')
                                ui.checkbox('Drive backwards to start', value=True).bind_value(self.system.weeding, 'drive_backwards_to_start') \
                                    .tooltip('Set the weeding automation to drive backwards to the start row at the end of the row')
                                ui.checkbox('Drive to start row', value=True).bind_value(self.system.weeding, 'drive_to_start') \
                                    .tooltip('Set the weeding automation to drive to the start of the row before starting the weeding')

                        ui.separator()
                        ui.markdown('Detector settings').style('color: #6E93D6')
                        with ui.row():
                            ui.number('Min. weed confidence', format='%.2f', value=0.8, step=0.05, min=0.0, max=1.0) \
                                .props('dense outlined') \
                                .classes('w-24') \
                                .bind_value(self.system.plant_locator, 'minimum_weed_confidence') \
                                .tooltip('Set the minimum weed confidence for the weeding automation')
                            ui.number('Min. crop confidence', format='%.2f', value=0.4, step=0.05, min=0.0, max=1.0) \
                                .props('dense outlined') \
                                .classes('w-24') \
                                .bind_value(self.system.plant_locator, 'minimum_crop_confidence') \
                                .tooltip('Set the minimum crop confidence for the weeding automation')
                        ui.separator()
                        ui.markdown('Tool settings').style('color: #6E93D6')
                        with ui.row():
                            if self.system.field_friend.tool == 'tornado':
                                ui.number('Tornado angle', format='%.0f', value=180, step=1, min=1, max=180) \
                                    .props('dense outlined suffix=°') \
                                    .classes('w-24') \
                                    .bind_value(self.system.weeding, 'tornado_angle') \
                                    .tooltip('Set the angle for the tornado drill')
                            elif self.system.field_friend.tool in ['weed_screw', 'dual_mechanism']:
                                ui.number('Drill depth', value=0.02, format='%.2f', step=0.01,
                                          min=self.system.field_friend.z_axis.max_position, max=self.system.field_friend.z_axis.min_position*-1) \
                                    .props('dense outlined suffix=°') \
                                    .classes('w-24') \
                                    .bind_value(self.system.weeding, 'weed_screw_depth') \
                                    .tooltip('Set the drill depth for the weeding automation')
                                ui.number('Crop safety distance', value=0.01, step=0.001, min=0.001, max=0.05, format='%.3f') \
                                    .props('dense outlined suffix=m') \
                                    .classes('w-24') \
                                    .bind_value(self.system.weeding, 'crop_safety_distance') \
                                    .tooltip('Set the crop safety distance for the weeding automation')
                        ui.separator()
                        ui.markdown('Workflow settings').style('color: #6E93D6')
                        with ui.row():
                            ui.checkbox('Only monitoring') \
                                .bind_value(self.system.weeding, 'only_monitoring') \
                                .tooltip('Set the weeding automation to only monitor the field')
                            if self.system.field_friend.tool == 'tornado':
                                ui.checkbox('With punch check', value=True) \
                                    .bind_value(self.system.puncher, 'with_punch_check') \
                                    .tooltip('Set the weeding automation to check for punch')
                                ui.checkbox('Drill 2x with open torando', value=False,) \
                                    .bind_value(self.system.weeding, 'drill_with_open_tornado') \
                                    .tooltip('Set the weeding automation to drill a second time with open tornado')
                                ui.checkbox('Drill between crops', value=False) \
                                    .bind_value(self.system.weeding, 'drill_between_crops') \
                                    .tooltip('Set the weeding automation to drill between crops')
                            elif self.system.field_friend.tool == 'dual_mechanism':
                                ui.checkbox('Drilling', value=False).bind_value(self.system.weeding, 'with_drilling') \
                                    .tooltip('Set the weeding automation to with drill')
                                ui.checkbox('Chopping', value=False) \
                                    .bind_value(self.system.weeding, 'with_chopping') \
                                    .tooltip('Set the weeding automation to with chop')
                                ui.checkbox('Chop if no crops', value=False) \
                                    .bind_value(self.system.weeding, 'chop_if_no_crops') \
                                    .tooltip('Set the weeding automation to chop also if no crops seen')
                        ui.separator()
                        ui.markdown('**Driver settings**').style('color: #6E93D6')
                        with ui.row():
                            ui.number('linear_speed_on_row', value=0.5, step=0.005, min=0.015, format='%.2f') \
                                .props('dense outlined suffix=m/s') \
                                .classes('w-24') \
                                .bind_value(self.system.weeding, 'linear_speed_on_row') \
                                .tooltip('Set the linear speed on row for the weeding automation')
                            ui.number('linear_speed_between_rows', value=0.5, step=0.01, min=0.03, format='%.2f') \
                                .props('dense outlined suffix=m/s') \
                                .classes('w-24') \
                                .bind_value(self.system.weeding, 'linear_speed_between_rows') \
                                .tooltip('Set the linear speed between rows for the weeding automation')
                            ui.number('angular_speed_on_row', value=0.5, step=0.01, min=0.03, format='%.2f') \
                                .props('dense outlined suffix=°/s') \
                                .classes('w-24') \
                                .bind_value(self.system.weeding, 'angular_speed_on_row') \
                                .tooltip('Set the angular speed on row for the weeding automation')
                            ui.number('angular_speed_between_rows', value=0.5, step=0.01, min=0.03, format='%.2f') \
                                .props('dense outlined suffix=°/s') \
                                .classes('w-24') \
                                .bind_value(self.system.weeding, 'angular_speed_between_rows') \
                                .tooltip('Set the angular speed between rows for the weeding automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='monitoring'):
                        with ui.column():
                            self.with_field_planning_monitor = ui.checkbox('Use field planning', value=True) \
                                .bind_value(self.system.monitoring, 'use_field_planning') \
                                .tooltip('Set the monitoring automation to use the field planning with GNSS')

                        with ui.row().bind_visibility_from(self.with_field_planning_monitor, 'value', value=True):
                            self.show_start_row()
                            self.show_end_row()
                            ui.number('Min. turning radius', format='%.2f',
                                      value=0.5, step=0.05, min=0.05, max=2.0) \
                                .props('dense outlined suffix=m').classes('w-30') \
                                .bind_value(self.system.monitoring, 'minimum_turning_radius') \
                                .tooltip('Set the turning radius for the monitoring automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='collecting (demo)'):
                        with ui.row():
                            ui.number('Drill angle', format='%.0f', value=100, step=1, min=1, max=180) \
                                .props('dense outlined suffix=°').classes('w-24') \
                                .bind_value(self.system.coin_collecting, 'angle') \
                                .tooltip('Set the drill depth for the weeding automation')
                            ui.checkbox('with drilling', value=True) \
                                .bind_value(self.system.coin_collecting, 'with_drilling')








                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='mowing'):
                        with ui.row():
                            ui.number('Padding', value=0.5, step=0.1, min=0.0, format='%.1f') \
                                .props('dense outlined suffix=m').classes('w-24') \
                                .bind_value(system.mowing, 'padding') \
                                .tooltip('Set the padding for the mowing automation')
                            ui.number('Lane distance', value=0.5, step=0.1, min=0.0, format='%.1f') \
                                .props('dense outlined suffix=m') \
                                .classes('w-24').bind_value(system.mowing,   'lane_distance') \
                                .tooltip('Set the lane distance for the system. automation')
                            ui.number('Number of outer lanes', value=3, step=1, min=3, format='%.0f') \
                                .props('dense outlined').classes('w-28') \
                                .bind_value(system.mowing, 'number_of_outer_lanes') \
                                .tooltip('Set the number of outer lanes for the mowing automation')
                            ui.number('Min. turning radius', format='%.2f', value=0.5, step=0.05, min=0.1, max=2.0) \
                                .props('dense outlined suffix=m') \
                                .classes('w-32') \
                                .bind_value(self.system.mowing, 'minimum_turning_radius') \
                                .tooltip('Set the turning radius for the mowing automation')