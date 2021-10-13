function myTank() {
class AdvancedState {
  constructor(state) {
    this.state = state;
    this.absoluteRadarAngle = Math.deg.normalize(
      this.state.radar.angle + this.state.angle
    );
    this.position = { x: state.x, y: state.y };
  }

  orientationString() {
    return `${this.state.x}${this.state.y}${this.state.angle}${this.state.radar.angle}`;
  }

  getPointAtDistanceAlongRadar(distance) {
    return {
      x:
        this.state.x +
        Math.cos((this.absoluteRadarAngle * Math.PI) / 180) * distance,
      y:
        this.state.y +
        Math.sin((this.absoluteRadarAngle * Math.PI) / 180) * distance,
    };
  }
}

const BATTLEFIELD_WIDTH = 850;
const BATTLEFIELD_HEIGHT = 550;
const SOUTH = 90;
const NORTH = -90;
const WEST = -180;
const EAST = 0;
const TANK_WIDTH=36;

var Constants = /*#__PURE__*/Object.freeze({
  __proto__: null,
  BATTLEFIELD_WIDTH: BATTLEFIELD_WIDTH,
  BATTLEFIELD_HEIGHT: BATTLEFIELD_HEIGHT,
  SOUTH: SOUTH,
  NORTH: NORTH,
  WEST: WEST,
  EAST: EAST,
  TANK_WIDTH: TANK_WIDTH
});

Number.prototype.isWhole = function (tolerance = 0.00000001) {
  return Math.abs(Math.round(this) - this) < tolerance;
};

// Track when we see the same X or Y values repeat on subsequent wall detections,
// use that to say a wall exists at those X or Y values and then based on which
// way the radar is pointing, calculate the origin point from them.
//
class OriginFinder {
  constructor() {
    this.origin = { x: undefined, y: undefined };
    this.lastX = undefined;
    this.lastY = undefined;
    this.lastOrientation = undefined;
  }

  update(advancedState) {
    if (this.origin.x && this.origin.y) { return }

    // If we don't see a wall, there's no info to work with
    let wallDistance = advancedState.state.radar.wallDistance;
    if (!wallDistance) { return }

    let orientationChanged =
      this.lastOrientation && this.lastOrientation != advancedState.orientationString();

    this.lastOrientation = advancedState.orientationString();

    let point = advancedState.getPointAtDistanceAlongRadar(wallDistance);

    if (!this.origin.x && point.x.isWhole()) {
      if (this.lastX && this.lastX == Math.round(point.x) && orientationChanged) {
        this.origin.x =
          this.lastX -
          (Math.abs(advancedState.absoluteRadarAngle) >= 90 ? 0 : BATTLEFIELD_WIDTH);
      }
      this.lastX = Math.round(point.x);
    }

    if (!this.origin.y && point.y.isWhole()) {
      if (this.lastY && this.lastY == Math.round(point.y) && orientationChanged) {
        this.origin.y =
          this.lastY -
          (advancedState.absoluteRadarAngle > 0 ? BATTLEFIELD_HEIGHT : 0);
      }
      this.lastY = Math.round(point.y);
    }
  }
}

class Autopilot {
  constructor() {
    this.originFinder = new OriginFinder();
    this.origin = this.originFinder.origin;
    this.advancedState = undefined;
    this.path = [];
    this.nextPosition = undefined;
  }

  update(state, control) {
    this.state = state;
    this.advancedState = new AdvancedState(state);
    this.control = control;

    this.originFinder.update(this.advancedState);
  }

  isOriginKnown() {
    return this.origin.x && this.origin.y;
  }

  setOrigin(x, y) {
    this.origin.x = x;
    this.origin.y = y;
  }

  lookEverywhere() {
    this.control.RADAR_TURN = 1;
  }

  lookAtEnemy(enemy) {
    let targetAngle = Math.deg.atan2(enemy.y - this.state.y, enemy.x - this.state.x);
    let radarAngle = Math.deg.normalize(targetAngle - this.state.angle);
    let angleDiff = Math.deg.normalize(radarAngle - this.state.radar.angle);
    this.control.RADAR_TURN = angleDiff;
  }

  isWallCollisionImminent(inTicks = 3) {
    if (this.state.collisions.wall) { return true }

    if (!this.isOriginKnown()) { return undefined }

    let positionInTicks = this.extrapolatedOuterPosition(inTicks);

    return (
      positionInTicks.x <= this.origin.x ||
      positionInTicks.x >= this.origin.x + BATTLEFIELD_WIDTH ||
      positionInTicks.y <= this.origin.y ||
      positionInTicks.y >= this.origin.y + BATTLEFIELD_HEIGHT
    )
  }

  turnToAngle(angle) {
    angle = Math.deg.normalize(angle);
    if (this.state.angle == angle) {
      this.control.TURN = 0;
      return 0;
    }

    let diffAngleToGoal = (Math.deg.normalize(angle - this.state.angle));
    let turn = diffAngleToGoal/2.0;
    this.control.TURN = turn;
    return diffAngleToGoal;
  }

  turnToPoint(x,y, basedOnZeroOrigin = false) {
    if (!this.isOriginKnown() && basedOnZeroOrigin) {
      throw new "Cannot turn to point based on zero origin because the origin is not yet known";
    }

    let translatedX = x + (basedOnZeroOrigin ? this.origin.x : 0);
    let translatedY = y + (basedOnZeroOrigin ? this.origin.y : 0);
    let angle = Math.deg.atan2(translatedY - this.state.y, translatedX - this.state.x);
    return this.turnToAngle(angle);
  }

  moveToPoint(x,y, basedOnZeroOrigin = false) {
    this.control.THROTTLE = 0;
    let angleRemaining = this.turnToPoint(x, y, basedOnZeroOrigin);
    if (angleRemaining < 60) {
      this.control.THROTTLE = (60 - angleRemaining) / 60;
    }
  }

  // stop moving if about to hit wall (or hitting wall)

  moveAlongAngle(angle) {
    this.control.THROTTLE = 0;
    let angleRemaining = Math.abs(this.turnToAngle(angle));
    if (angleRemaining < 60) {
      this.control.THROTTLE = (60 - angleRemaining) / 60;
    }
  }

  loopOnPath(positions, basedOnZeroOrigin, tolerance = 50) {
    if (positions.length == 0) { return }

    if (this.path.length == 0) {
      this.path = positions;

      if (basedOnZeroOrigin) {
        if (!this.isOriginKnown()) {
          throw new "Cannot loop on positions based on zero origin because the origin is not yet known";
        }

        this.path.forEach((position) => {
          position.x += this.origin.x;
          position.y += this.origin.y;
        });
      }
    }

    if (!this.nextPosition) {
      this.nextPosition = this.path.shift();
      this.path.push(this.nextPosition);
    }

    let distanceRemaining = Math.distance(this.nextPosition.x, this.nextPosition.y, this.state.x, this.state.y);
    if (distanceRemaining <= tolerance) {
      this.nextPosition = this.path.shift();
      this.path.push(this.nextPosition);
    }

    this.moveToPoint(this.nextPosition.x, this.nextPosition.y);
  }

  stopLoopOnPath() {
    this.path = [];
    this.nextPosition = undefined;
    this.stop();
  }

  stop() {
    this.control.TURN = 0;
    this.control.THROTTLE = 0;
    this.control.BOOST = 0;
  }

  shootEnemy(enemy) {
    if (!enemy) { return }

    // predict position of moving target
    let bulletSpeed = 4;
    let distance = Math.distance(this.state.x, this.state.y, enemy.x, enemy.y);
    let bulletTime = distance / bulletSpeed;
    let target = Autopilot.extrapolatedPosition(enemy, enemy.angle, enemy.speed, bulletTime);

    // calculate desired direction of the gun
    let targetAngle = Math.deg.atan2(target.y - this.state.y, target.x - this.state.x);
    let gunAngle = Math.deg.normalize(targetAngle - this.state.angle);

    // point the gun at the target
    let angleDiff = Math.deg.normalize(gunAngle - this.state.gun.angle);
    this.control.GUN_TURN = 0.3 * angleDiff;

    // shoot when aiming at target
    if(Math.abs(angleDiff) < 2) {
      this.control.SHOOT = 0.1;
    }
  }

  extrapolatedPosition(inTicks) {
    return Autopilot.extrapolatedPosition(this.advancedState.position, this.state.angle, this.speed(), inTicks);
  }

  extrapolatedOuterPosition(inTicks) {
    let extrapolatedPosition = this.extrapolatedPosition(inTicks);
    return {
      x: extrapolatedPosition.x + Math.sqrt(2) * TANK_WIDTH / 2 * (Math.abs(this.state.angle) >= 90 ? -1 : 1 ),
      y: extrapolatedPosition.y + Math.sqrt(2) * TANK_WIDTH / 2 * (this.state.angle < 0 ? -1 : 1),
    }
  }

  speed() {
    return this.control.THROTTLE * 2 * (this.control.BOOST == 1 ? 2 : 1)
  }

  static extrapolatedPosition(startPosition, travelAngle, travelSpeed, inTicks) {
    return {
      x: startPosition.x + inTicks * travelSpeed * Math.cos(Math.deg2rad(travelAngle)),
      y: startPosition.y + inTicks * travelSpeed * Math.sin(Math.deg2rad(travelAngle))
    }
  }
}

  // rando data
  const autopilot = new Autopilot();
  // end rando data

  // start strategies 

  const updateAutopilot = (state, control) => {
    autopilot.update(state, control);
    return {};
  }

  const discoverOrigin = (state, control) => {
    if (autopilot.isOriginKnown()) {
      return {};
    }

    return {RADAR_TURN: 1};
  }

  const avoidCollidingWithWalls = (state, control) => {
    // TODO - use some number of ticks based on speed
    let positionInTicks = autopilot.extrapolatedOuterPosition(30);

    //const distanceRemaining = Math.distance(positionInTicks.x, positionInTicks.y, state.x, state.y);
    //const throttleInstruction = distanceRemaining < 10
    //  ? {THROTTLE: 0}
    //  : distanceRemaining < 20
    //    ? {THROTTLE: 0.5}
    //    : {}
    const throttleInstruction = {THROTTLE: 0}

    if (positionInTicks.x <= autopilot.origin.x) {
      return {TURN: state.angle > 0 ? -1 : 1, ...throttleInstruction};
    }
    if (positionInTicks.x >= autopilot.origin.x + Constants.BATTLEFIELD_WIDTH) {
      return {TURN: state.angle > 0 ? 1 : -1, ...throttleInstruction};
    }
    if (positionInTicks.y <= autopilot.origin.y) {
      return {TURN: Math.abs(state.angle) < 90 ? 1 : -1, ...throttleInstruction};
    }
    if (positionInTicks.y >= autopilot.origin.y + Constants.BATTLEFIELD_HEIGHT) {
      return {TURN: Math.abs(state.angle) < 90 ? -1 : 1, ...throttleInstruction};
    }
  };

  const alwaysBeDriving = (state, control) => ({
    THROTTLE: 1,
  });
  
  const alwaysBeShooting = (state, control) => ({
    GUN_TURN: 0,
    SHOOT: 0.1,
  });

  const moveRandomly = (state, control) => ({
    TURN: Math.floor(Math.random() * 20) == 2 ? Math.random()*2 - 1 : control.TURN,
    THROTTLE: 0,
  });

  // end strategies

  tank.init(function(settings, info) {

  });

  var tick = 0;
  tank.loop(function(state, control) {
    tick++;

    const instructions = [
      updateAutopilot,
      discoverOrigin,
      avoidCollidingWithWalls,
      alwaysBeShooting,
      alwaysBeDriving,
      moveRandomly
    ]
      .map(strategy => strategy(state, control))
      .reduce((result, action) => ({...action, ...result}), {})
    ;

    control.THROTTLE = instructions.THROTTLE || 0;
    control.TURN = instructions.TURN || 0;
    control.RADAR_TURN = instructions.RADAR_TURN || 0;
    control.SHOOT = instructions.SHOOT || 0;
    control.GUN_TURN = instructions.GUN_TURN || 0;
  });
  

}

window.myTank = myTank
