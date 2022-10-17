function cleanup_vrep(vrep, id)
  fprintf('Closing connection %d.\n', id);
  vrep.simxStopSimulation(id, vrep.simx_opmode_oneshot_wait);
  vrep.simxFinish(id);
  vrep.delete(); % You may need to comment this call if it crashed Matlab.
  disp('Program ended');
end

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)
