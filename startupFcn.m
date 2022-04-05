 % Code that executes after component creation
        function startupFcn(app)
            app.RUN = 0;
            app.NPOINTS = 300;
            app.r = 0;
            app.Kp = 0;
            app.Ki = 0;
            app.Kd = 0;
            delete(instrfind);
            app.comport = serial('COM6','BaudRate',115200);
            fopen(app.comport);
        end
        % Button pushed function: QUITButton
        function QUITButtonPushed(app, event)
            app.RUN = 0;
            fclose(app.comport);
            clear app.comport
            delete(app)
            
        end
        % Value changed function: DesiredTemperatureEditField
        function DesiredTemperatureEditFieldValueChanged(app, event)
            value = app.DesiredTemperatureEditField.Value;
            app.r = value;
        end
        % Value changed function: KpEditField
        function KpEditFieldValueChanged(app, event)
            value = app.KpEditField.Value;
            app.Kp = value;
        end
        % Value changed function: KiEditField
        function KiEditFieldValueChanged(app, event)
            value = app.KiEditField.Value;
            app.Ki = value;
        end
        % Value changed function: KdEditField
        function KdEditFieldValueChanged(app, event)
            value = app.KdEditField.Value;
            app.Kd = value;
        end
        
          % Button pushed function: RUNButton
        function RUNButtonPushed(app, event)
            i = 0;
            d = [];
            t = [];
        if app.RUN == 0
            app.RUN = 1;
            % change the string on the button to STOP
            app.RUNButton.Text = 'STOP';
            while app.RUN == 1
                % send set point to the MCU
                fprintf(app.comport,'%s',app.r);
                % compute current time
                time = i*0.094;
                % fetch data as single 8-bit byte               
                   data = fread(app.comport,1,'uint8');
                % send PID values to the MCU
                fprintf(app.comport,'%s',app.Kp);
                fprintf(app.comport,'%s',app.Ki);
                fprintf(app.comport,'%s',app.Kd);
                if i<app.NPOINTS
                    i = i+1;
                    d(i) = data;
                    t(i) = time;
                else
                    i = i+1;
                    d(1:app.NPOINTS-1) = d(2:app.NPOINTS);
                    d(app.NPOINTS) = data;
                    t(1:app.NPOINTS-1) = t(2:app.NPOINTS);
                    t(app.NPOINTS) = time;
                end
                % plot the data on the screen
                plot(app.UIAxes,t,d)
                ylim(app.UIAxes,[0 100])
                % use drawnow to update the figure
                drawnow
            end
        else
            app.RUN = 0;
            % change the string on the button to RUN
            app.RUNButton.Text = 'RUN';
        end
        end
        
        function resetInteractions(app, event)
            % This function resets the states of the toggle tools that
            % impact user interactions.  It also resets the figure interactions.
             
            % Find all tools to reset.  Exclude the tool associated
            % with the event.
            interactiveTools = [app.ZoomIn, app.ZoomOut, app.uitoggletool3];
            interactiveTools(event.Source == interactiveTools) = [];
             
            % Set the state of the tools to 'off'.
            [interactiveTools.State] = deal('off');
             
            % Set figure interactions to 'off'.
            datacursormode(app.UIFigure, 'off')
            rotate3d(app.UIFigure, 'off');
            pan(app.UIFigure, 'off');
            zoom(app.UIFigure,'off');
        end
        
function results = func5(app)
            
end
        
    