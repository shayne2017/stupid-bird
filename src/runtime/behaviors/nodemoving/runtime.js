var ls;
(function (ls) {
    var NodeMovingBehavior = (function (_super) {
        __extends(NodeMovingBehavior, _super);
        function NodeMovingBehavior() {
            _super.call(this);
            this.pathIndex = 0;
            this.moveSpeed = 60;
        }
        var d = __define,c=NodeMovingBehavior,p=c.prototype;
        p.onCreate = function () {
            this.moveSpeed = ls.eval_e(this.moveSpeed);
        };
        p.tick = function () {
            if (!this.isPathfinding)
                return;
            if (!this.path)
                return;
            var dt = this.inst.dt;
            var targetX = parseFloat(this.path[this.pathIndex][0]);
            var targetY = parseFloat(this.path[this.pathIndex][1]);
            var dx = targetX - this.inst.x;
            var dy = targetY - this.inst.y;
            var dist = Math.sqrt(dx * dx + dy * dy);
            var angle = Math.atan2(dy, dx);
            var oldX = this.inst.x;
            var oldY = this.inst.y;
            if (dist < this.moveSpeed * dt) {
                this.pathIndex++;
                this.dispatchEvent(new ls.TriggerEvent(ls.TriggerEvent.TRIGGER, this.onNodeMovingSegmentArrived));
                if (this.pathIndex >= this.path.length) {
                    this.isPathfinding = false;
                    this.inst.x = targetX;
                    this.inst.y = targetY;
                    this.dispatchEvent(new ls.TriggerEvent(ls.TriggerEvent.TRIGGER, this.onNodeMovingArrived));
                }
            }
            else {
                this.inst.x += Math.cos(angle) * this.moveSpeed * dt;
                this.inst.y += Math.sin(angle) * this.moveSpeed * dt;
            }
            var targetAngle = ls.MathUtils.angleTo(oldX, oldY, this.inst.x, this.inst.y);
            var newAngle = ls.MathUtils.angleRotate(this.inst.angle, targetAngle, 300);
            if (isNaN(newAngle))
                return;
            if (this.inst.angle != newAngle) {
                this.inst.angle = newAngle;
            }
        };
        p.isNodeMoving = function ($event) {
            return { instances: [this.inst], status: this.isPathfinding };
        };
        p.onNodeMovingReady = function ($event) {
            return { instances: [this.inst], status: true };
        };
        p.onNodeMovingArrived = function ($event) {
            return { instances: [this.inst], status: true };
        };
        p.onNodeMovingSegmentArrived = function ($event) {
            return { instances: [this.inst], status: true };
        };
        p.setMoveSpeed = function ($moveSpeed) {
            this.moveSpeed = ls.eval_e($moveSpeed);
            ls.assert(typeof this.moveSpeed !== "number", "NodeMovingBehaivor setMoveSpeed parameter type incorrect!!");
        };
        p.startNodePath = function ($pathStr) {
            if ($pathStr) {
                this.pathIndex = 0;
                var reg = /[\(\)]/g;
                $pathStr = $pathStr.replace(reg, "");
                var points = $pathStr.split(",");
                var paths = [];
                var pointIndex = 0;
                for (var i = 0, len = points.length; i < len; i++) {
                    if (i % 2 == 0) {
                        paths[pointIndex] = [];
                    }
                    paths[pointIndex].push(ls.eval_e(points[i]));
                    if (i % 2 == 1) {
                        pointIndex++;
                    }
                }
                this.path = paths;
                this.isPathfinding = true;
                this.dispatchEvent(new ls.TriggerEvent(ls.TriggerEvent.TRIGGER, this.onNodeMovingReady));
            }
        };
        d(p, "nodeIndex"
            ,function () {
                return this.pathIndex;
            }
        );
        d(p, "isMoving"
            ,function () {
                return this.isPathfinding;
            }
        );
        p.saveToJSON = function () {
            var o = _super.prototype.saveToJSON.call(this);
            o.moveSpeed = this.moveSpeed;
            return o;
        };
        p.loadFromJSON = function (o) {
            if (o) {
                this.moveSpeed = o.moveSpeed;
                _super.prototype.loadFromJSON.call(this, o);
            }
        };
        p.clone = function () {
            var bh = _super.prototype.clone.call(this);
            bh.moveSpeed = this.moveSpeed;
            return bh;
        };
        return NodeMovingBehavior;
    }(ls.BaseBehavior));
    ls.NodeMovingBehavior = NodeMovingBehavior;
    egret.registerClass(NodeMovingBehavior,'ls.NodeMovingBehavior');
    var IsNodeMovingEvent = (function (_super) {
        __extends(IsNodeMovingEvent, _super);
        function IsNodeMovingEvent() {
            _super.apply(this, arguments);
        }
        var d = __define,c=IsNodeMovingEvent,p=c.prototype;
        return IsNodeMovingEvent;
    }(ls.BaseEvent));
    ls.IsNodeMovingEvent = IsNodeMovingEvent;
    egret.registerClass(IsNodeMovingEvent,'ls.IsNodeMovingEvent');
    var OnNodeMovingReadyEvent = (function (_super) {
        __extends(OnNodeMovingReadyEvent, _super);
        function OnNodeMovingReadyEvent() {
            _super.apply(this, arguments);
        }
        var d = __define,c=OnNodeMovingReadyEvent,p=c.prototype;
        return OnNodeMovingReadyEvent;
    }(ls.BaseEvent));
    ls.OnNodeMovingReadyEvent = OnNodeMovingReadyEvent;
    egret.registerClass(OnNodeMovingReadyEvent,'ls.OnNodeMovingReadyEvent');
    var OnNodeMovingArrivedEvent = (function (_super) {
        __extends(OnNodeMovingArrivedEvent, _super);
        function OnNodeMovingArrivedEvent() {
            _super.apply(this, arguments);
        }
        var d = __define,c=OnNodeMovingArrivedEvent,p=c.prototype;
        return OnNodeMovingArrivedEvent;
    }(ls.BaseEvent));
    ls.OnNodeMovingArrivedEvent = OnNodeMovingArrivedEvent;
    egret.registerClass(OnNodeMovingArrivedEvent,'ls.OnNodeMovingArrivedEvent');
    var OnNodeMovingSegmentArrivedEvent = (function (_super) {
        __extends(OnNodeMovingSegmentArrivedEvent, _super);
        function OnNodeMovingSegmentArrivedEvent() {
            _super.apply(this, arguments);
        }
        var d = __define,c=OnNodeMovingSegmentArrivedEvent,p=c.prototype;
        return OnNodeMovingSegmentArrivedEvent;
    }(ls.BaseEvent));
    ls.OnNodeMovingSegmentArrivedEvent = OnNodeMovingSegmentArrivedEvent;
    egret.registerClass(OnNodeMovingSegmentArrivedEvent,'ls.OnNodeMovingSegmentArrivedEvent');
})(ls || (ls = {}));
