function clothoidBasedParams = clothoidBasedControllerParams(kus,look_ahead)

    % ----------------------------------------------------------------
    %% Function purpose: define the clothoid-based lateral controller parameters
    % ----------------------------------------------------------------
    
    % Look ahead distance (in meters) used by the controller
    clothoidBasedParams.lookAhead = look_ahead; 
    
    % Understeering gradient
    clothoidBasedParams.Kus = kus;  % REPLACE THIS WITH THE CORRECT VALUE!!!! 

end

