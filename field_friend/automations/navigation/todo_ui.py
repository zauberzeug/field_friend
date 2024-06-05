                        ui.markdown('Field settings').style('color: #6E93D6')
                        with ui.row():

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

                        ui.separator()

                        ui.markdown('Workflow settings').style('color: #6E93D6')
                        with ui.row():
                            if self.system.field_friend.tool == 'dual_mechanism':
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
                            







