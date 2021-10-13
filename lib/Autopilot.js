import { AdvancedState } from "./AdvancedState";
import { Constants } from "./main";
import { OriginFinder } from "./OriginFinder";

export class Autopilot {
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
      positionInTicks.x >= this.origin.x + Constants.BATTLEFIELD_WIDTH ||
      positionInTicks.y <= this.origin.y ||
      positionInTicks.y >= this.origin.y + Constants.BATTLEFIELD_HEIGHT
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
    let angle = Math.deg.atan2(translatedY - this.state.y, translatedX - this.state.x)
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
        })
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

    this.moveToPoint(this.nextPosition.x, this.nextPosition.y)
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
    const instruction = {}

    // predict position of moving target
    let bulletSpeed = 4;
    let distance = Math.distance(this.state.x, this.state.y, enemy.x, enemy.y)
    let bulletTime = distance / bulletSpeed;
    let target = Autopilot.extrapolatedPosition(enemy, enemy.angle, enemy.speed, bulletTime)

    // calculate desired direction of the gun
    let targetAngle = Math.deg.atan2(target.y - this.state.y, target.x - this.state.x);
    let gunAngle = Math.deg.normalize(targetAngle - this.state.angle);

    // point the gun at the target
    let angleDiff = Math.deg.normalize(gunAngle - this.state.gun.angle);
    instruction.GUN_TURN = this.control.GUN_TURN = 0.3 * angleDiff;

    // shoot when aiming at target
    if(Math.abs(angleDiff) < 2) {
      instruction.SHOOT = this.control.SHOOT = 0.1;
    }

    return instruction;
  }

  extrapolatedPosition(inTicks) {
    return Autopilot.extrapolatedPosition(this.advancedState.position, this.state.angle, this.speed(), inTicks);
  }

  extrapolatedOuterPosition(inTicks) {
    let extrapolatedPosition = this.extrapolatedPosition(inTicks);
    return {
      x: extrapolatedPosition.x + Math.sqrt(2) * Constants.TANK_WIDTH / 2 * (Math.abs(this.state.angle) >= 90 ? -1 : 1 ),
      y: extrapolatedPosition.y + Math.sqrt(2) * Constants.TANK_WIDTH / 2 * (this.state.angle < 0 ? -1 : 1),
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
