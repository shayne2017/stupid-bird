var ls;
(function (ls) {
    var PushoutSolidBehavior = (function (_super) {
        __extends(PushoutSolidBehavior, _super);
        function PushoutSolidBehavior() {
            _super.call(this);
        }
        var d = __define,c=PushoutSolidBehavior,p=c.prototype;
        p.onCreate = function () {
            this.obstacleMode = ls.eval_e(this.obstacleMode);
            this.obstacles = [];
            this.inst.pushoutSolid = true;
        };
        p.tick = function () {
            if (this.obstacleMode === 0) {
                var collobj = ls.CollisionUtils.testOverlapSolid(this.inst);
                if (collobj) {
                    ls.CollisionUtils.pushOutSolidNearest(this.inst);
                }
            }
            else {
                if (this.obstacles.length === 0)
                    return;
                var firstOverlapInst = this.getFirstOverlapInst();
                if (firstOverlapInst) {
                    ls.CollisionUtils.pushOutSolidNearest(this.inst);
                }
            }
        };
        p.getFirstOverlapInst = function () {
            if (this.obstacles) {
                for (var i = 0, l = this.obstacles.length; i < l; i++) {
                    if (ls.CollisionUtils.testOverlap(this.inst, this.obstacles[i])) {
                        return this.obstacles[i];
                    }
                }
            }
            return null;
        };
        p.addObstacle = function (object) {
            var index = this.obstacles.indexOf(object);
            if (index === -1) {
                object.customSolidEnabled = true;
                this.obstacles.push(object);
            }
        };
        p.clearObstacles = function () {
            for (var i = 0, l = this.obstacles.length; i < l; i++) {
                this.obstacles[i].customSolidEnabled = false;
            }
            this.obstacles.length = 0;
        };
        p.clone = function () {
            var bh = _super.prototype.clone.call(this);
            bh.obstacleMode = this.obstacleMode;
            return bh;
        };
        return PushoutSolidBehavior;
    }(ls.BaseBehavior));
    ls.PushoutSolidBehavior = PushoutSolidBehavior;
    egret.registerClass(PushoutSolidBehavior,'ls.PushoutSolidBehavior');
})(ls || (ls = {}));
