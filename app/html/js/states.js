define([

    'angularUiRouter',
    '/apps/proactive-mobility/js/core/module.js',
    '/apps/proactive-mobility/js/core/controllers/base.js',

], function() {

    'use strict';

    angular.module('app.states.proactive-mobility', [
        'ui.router',
        'app.proactive-mobility'
    ]).config([
            '$stateProvider',
            '$urlRouterProvider',
            '$urlMatcherFactoryProvider',
            function($stateProvider,
                     $urlRouterProvider,
                     $urlMatcherFactoryProvider
                ) {

        $stateProvider
            .state('menu.root.proactive-mobility', {
                url: '/proactive-mobility',
                views: {
                    'content@menu': {
                        templateUrl: '/apps/proactive-mobility/partials/base.html',
                        controller: 'ChargingController'
                    }
                }
            });
        }
    ]);
});
