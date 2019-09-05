'use strict';
define([
    '/apps/proactive-mobility/js/core/module.js'
], function (controllers) {
    controllers.controller('ChargingController',
                           ['$rootScope',
                            '$scope',
                            '$q',
                            'sPref',
                            'sSystem',
                            '$translatePartialLoader',
                            '$translate',
                            function($rootScope,
                                     $scope,
                                     $q,
                                     sPref,
                                     sSystem,
                                     $translatePartialLoader,
                                     $translate) {
        /* INIT */
        $scope.prefValues = {
        }
        /* Defining pref params: */
        var DOMAIN = "com.softbankrobotics.proactive-mobility";
        var valueParams = {
            'IsActive': {
                default: false,
            },
            'Technology': {
                default: 'Slam',
            },
            'MaxDistance': {
                default: 3.0,
                min: 0.5,
                max: 3.0,
                step: 0.5,
            },
            'MaxAngle': {
                default: 180.0,
                min: 60.0,
                max: 360.0,
                step: 10.0,
            },
            'GoHomeTimeout': {
                default: 10.0,
                min: 0.5,
                max: 100000,
                step: 1.0,
            },
        }
        $scope.technologyEntries = [
            {label: 'Slam'},
            {label: 'Aruco'},
            {label: 'Pod'},
        ];
        // Inner helpers for prefs
        function checkRangeValidity(key) {
            var value = parseFloat($scope.prefValues[key]);
            var defaultValue = valueParams[key].default;
            console.log(["checking range", key, value, defaultValue]);
            if (isNaN(value)) {
                value = defaultValue;
            } else if (value < valueParams[key].min) {
                value = defaultValue;
            } else if (value > valueParams[key].max) {
                value = defaultValue;
            }
            console.log(["checked range", key, value, defaultValue]);
            // Else, the value is in the range, fine!
            $scope.prefValues[key] = value;
        }

        var prefUpdateRequest = 0;
        function handlePrefUpdateNeeded() {
            // Wait a bit then maybe update pref.
            // the result if that prefs are udpated two seconds
            // after the last time someone changed them.
            prefUpdateRequest += 1;
            var thisRequest = prefUpdateRequest;
            setTimeout(function() {
                if (thisRequest == prefUpdateRequest) {
                    sPref.updatePref();
                }
            }, 2000);
        }

        function setPref(key, value) {
            $scope.prefValues[key] = value;
            // Coerce to string
            sPref.setPref(DOMAIN, key, "" + value);
            handlePrefUpdateNeeded();
        }

        // Pref manipulation from the view

        $scope.setIsActive = function(isActive) {
            $scope.isActive = isActive;
            // Convert bool to on/off string string
            if (isActive) {
                setPref("IsActive", "On");
            } else {
                setPref("IsActive", "Off");
            }
        };

        $scope.setTechnology = function(technologyEntry) {
            console.log(technologyEntry);
            setPref("Technology", technologyEntry.label);
            setMaxAngleForTechnology(technologyEntry.label)
        };

        $scope.canIncrease = function(key) {
            return $scope.prefValues[key] < valueParams[key].max;
        }

        $scope.canDecrease = function(key) {
            return $scope.prefValues[key] > valueParams[key].min;
        }

        $scope.increase = function(key) {
            if ($scope.canIncrease(key)) {
                setPref(key, $scope.prefValues[key] + valueParams[key].step);
            }
        }

        $scope.decrease = function(key) {
            if ($scope.canDecrease(key)) {
                setPref(key, $scope.prefValues[key] - valueParams[key].step);
            }
        }

        function setMaxAngleForTechnology(technology) {
            console.log('setMaxAngleForTechnology', technology)
            if (technology === 'Pod') {
                valueParams['MaxAngle'].max = 180;
                if ($scope.prefValues['MaxAngle'] > 180) {
                    $scope.prefValues['MaxAngle'] = 180;
                }
            } else {
                valueParams['MaxAngle'].max = 360;
            }
        }

        function capitalize(string) {
            return string.charAt(0).toUpperCase() + string.slice(1).toLowerCase();
        }

        function formatTechnology(prefValue) {
            if (prefValue) {
                prefValue = capitalize(prefValue);
                if (/Slam|Aruco|Pod/.test(prefValue)) {
                    return prefValue;
                }
            }
            // Invalid or no value, return default
            return valueParams["Technology"].default; // which is "Slam"
        }
        function isActiveToBool(prefValue) {
            if (!prefValue) {
                return valueParams["IsActive"].default; // which is false
            } else {
                prefValue = ("" + prefValue).toLowerCase(); // Coerce to string
                if (/1|true|on|yes/.test(prefValue)) {
                    return true;
                } else if (/0|false|off|no/.test(prefValue)) {
                    return false;
                } else {
                    return valueParams["IsActive"].default; // which is false
                }
            }
        }
        // Request the prefs
        $q.all([sPref.getPref(DOMAIN, "IsActive"),
                sPref.getPref(DOMAIN, "Technology"),
                sPref.getPref(DOMAIN, "MaxDistance"),
                sPref.getPref(DOMAIN, "MaxAngle"),
                sPref.getPref(DOMAIN, "GoHomeTimeout"),
               ]).then(function(data){
            // Load the prefs in model
            $scope.prefValues = {
                'IsActive': data[0],
                'Technology': formatTechnology(data[1]),
                'MaxDistance': data[2],
                'MaxAngle': data[3],
                'GoHomeTimeout': data[4],
            }

            // Now ensure they have the right format.
            // TODO: handle formatting
            $scope.isActive = isActiveToBool($scope.prefValues['IsActive']);
            // TODO: handle formatting
            $scope.technologyEntry = {
                label: $scope.prefValues['Technology'],
            };

            setMaxAngleForTechnology($scope.technologyEntry.label);

            checkRangeValidity('MaxDistance');
            checkRangeValidity('MaxAngle');
            checkRangeValidity('GoHomeTimeout');

            // We're done with prefs, now load translations
            $translatePartialLoader.addPart('/apps/proactive-mobility/lang');
            // Now remove the loading animation, and actually show the page
            $translate.refresh().then(function(){
                $rootScope.pageLoaded = true;
            });
        });
    }]);
});
