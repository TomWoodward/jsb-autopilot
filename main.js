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
    const instruction = {};

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
  const commandMemory = new Map();
  const autopilot = new Autopilot();
  const ticksToRotateRadar = 60;
  const enemies = {};
  const whosShootingWho = {};
  let lastKnownEnemy;
  const friendlies = {};
  var avoidingWalls = 0;
  var tick = 0;
  var myId;

  // end rando data

  function sortByKey(array, key) {
    return array.sort((a, b) => {
      let x = a[key];
      let y = b[key];

      return x < y ? -1 : x > y ? 1 : 0;
    });
  }

  const crazyIvan = (state, control) => {
    return {
      until: tick + 15,
      command: {
        TURN: Math.random() > 0.5 ? 1 : 0,
        BOOST: 1,
        THROTTLE: Math.ceil(control.THROTTLE * -1)
      }
    };
  };

  const whoShouldIShoot = (state, control) => {
    const enemyIds = Object.keys(enemies);
    let enemyDist = [];
    if (enemyIds.length > 0) {
      for (let e in enemyIds) {
        eNow = enemies[enemyIds[e]];
        enemyDist.push({
          health: eNow.energy,
          distance: Math.distance(eNow.x, eNow.y, state.x, state.y),
          id: eNow.id,
        });
      }
    }
    sortByKey(enemyDist, "health");
    const weakestEnemy = enemyIds.length > 0 && enemies[enemyDist[0].id];

    if (weakestEnemy.energy < 40) {
      return weakestEnemy;
    }

    sortByKey(enemyDist, "distance");
    const closesetEnemy = enemyIds.length > 0 && enemies[enemyDist[0].id];

    return closesetEnemy;
  }

  // start strategies

  const discoverOrigin = (state, control) => {
    if (autopilot.isOriginKnown()) {
      return;
    }

    return {command: {RADAR_TURN: 1}};
  }

  const trackNearbyEnemies = (state, control) => {
    if (state.radar.enemy) {
      enemies[state.radar.enemy.id] = {
        removeAfter: tick + (ticksToRotateRadar / 3),
        ...state.radar.enemy
      };
      control.OUTBOX.push({type: 'enemy', enemy: enemies[state.radar.enemy.id]})
    }

    (state.radio.inbox || [])
      .filter(message => message.type === 'enemy')
      .forEach(message => enemies[message.enemy.id] = message.enemy)
    ;

    for (const key of Object.keys(enemies)) {
      if (enemies[key].removeAfter < tick) {
        delete enemies[key]
      }
    }
  };

  const trackNearbyFriendlies = (state, control) => {
    if (state.radar.ally) {
      friendlies[state.radar.ally.id] = {
        removeAfter: tick + ticksToRotateRadar,
        ...state.radar.ally
      };
      control.OUTBOX.push({type: 'friendly', friendly: friendlies[state.radar.ally.id]})
    }

    control.OUTBOX.push({type: 'friendly', friendly: {
        x: state.x,
        y: state.y,
        id: myId,
        angle: state.angle,
      }
    });

    (state.radio.inbox || [])
      .filter(message => message.type === 'friendly')
      .forEach(message => friendlies[message.friendly.id] = {...friendlies[message.friendly.id], ...message.friendly})
    ;

    for (const key of Object.keys(friendlies)) {
      if (friendlies[key].removeAfter < tick) {
        // angle is the only thing that gets stale, everything else will be re-added every tick
        delete friendlies[key].angle
      }
    }
  };

  const surroundEnemies = (state, control) => {
    (state.radio.inbox || [])
      .filter(message => message.type === 'shooting')
      .forEach(message => whosShootingWho[message.me] = message.target)
    ;

    const imShootingId = whosShootingWho[String(myId)];
    const imShootingEnemy = imShootingId && enemies[imShootingId];

    if (!imShootingEnemy) {
      return;
    }

    const myAngle = Math.deg.atan2(imShootingEnemy.x - state.x, imShootingEnemy.y - state.y);

    const alliesShootingSameWithAngle = Object.entries(whosShootingWho)
      .filter(([friendlyId, imShootingEnemyId]) => friendlyId != myId && imShootingEnemyId === imShootingId)
      .map(([friendlyId]) => {
        return friendlies[String(friendlyId)]
      })
      .map(friendly => {
        const angle = Math.deg.atan2(imShootingEnemy.x - friendly.x, imShootingEnemy.y - friendly.y);
        const diff = Math.deg.normalize(myAngle - angle);
        return {
          friendly,
          angle,
          diff,
          diffAbs: Math.abs(diff),
        };
      })
    ;

    sortByKey(alliesShootingSameWithAngle, 'diffAbs');

    const closestAngle = alliesShootingSameWithAngle[0];

    if (!closestAngle) {
      return;
    }


    const directionToMove = closestAngle['diff'] > 0 ? -1 : 1;
    const idealAngle = Math.deg.atan2(imShootingEnemy.y - state.y, imShootingEnemy.x - state.x) - (90 * directionToMove);

    return {
      TURN: Math.deg.normalize(idealAngle - state.angle)
    }
  };

  const lockRadarOnNearbyEnemies = (state, control) => {
    const imShootingId = whosShootingWho[String(myId)];
    const enemy = imShootingId && enemies[imShootingId];

    if (enemy) {
      autopilot.lookAtEnemy(enemy);
    }
  };

  const alwaysBeScanning = (state, control) => {
    return {command: {RADAR_TURN: 1}};
  }

  const counterGunTurn = (state, control) => {
    return { command: { GUN_TURN: -1 } };
  };

  const shootAtVisibleTanks = (state, control) => {
    const enemy = whoShouldIShoot(state, control);

    whosShootingWho[String(myId)] = enemy.id;
    control.OUTBOX.push({type: 'shooting', target: enemy.id, me: myId})

    if (enemy) {
      const instruction = autopilot.shootEnemy(enemy);
      const distance = Math.distance(enemy.x, enemy.y, state.x, state.y);
      if (
        distance < 50 ||
        (instruction.SHOOT && distance < 180 && enemy.speed < 1)
      ) {
        instruction.SHOOT = 1;
      }
      return {command: instruction};
    }
  };

  const dontShootAllies = (state, control) => {

    const allyInFront = Object.values(friendlies).find(ally => {
      const targetAngle = Math.deg.atan2(ally.y - state.y, ally.x - state.x);
      const angleDiff = Math.deg.normalize(targetAngle - state.gun.angle);

      return Math.abs(angleDiff) < 15;
    });

    if (allyInFront) {
      return {command: {SHOOT: 0}}
    }
  }

  const ramJamro = (state, control) => {
    if (state.energy < 50) {
      return
    }

    const enemiesWithInfo = Object.values(enemies)
      .map(enemy => {
        const targetAngle = Math.deg.atan2(enemy.y - state.y, enemy.x - state.x);
        const angleDiff = Math.deg.normalize(targetAngle - state.angle);
        const distance = Math.distance(enemy.x, enemy.y, state.x, state.y);

        return {
          enemy,
          angleDiff,
          targetAngle,
          distance,
        };
      })
      .filter(({angleDiff, distance}) =>
        angleDiff < 30 && distance < 80 ||
        angleDiff < 60 && distance < 70 ||
        angleDiff < 80 && distance < 60 ||
        angleDiff < 100 && distance < 50 ||
        angleDiff < 120 && distance < 40 ||
        angleDiff < 180 && distance < 30
      )
    ;

    sortByKey(enemiesWithInfo, 'distance');

    const {enemy, angleDiff, distance} = enemiesWithInfo[0] || {enemy: undefined};

    if (state.collisions.enemy) {
      return [
        {command: {THROTTLE: -1, BOOST: 1}, until: tick + 5},
        {command: {THROTTLE: -1, BOOST: 0}, until: tick + 20}
      ]
    } else if (enemy) {
      console.log('RAMMING SPEED');
      return {command: {TURN: angleDiff, THROTTLE: 1, BOOST: distance < 40 ? 1 : 0}}
    }
  };

  const dodgeBullets = (state, control) => {
    if (state.radar.bullets.filter(bullet => bullet.damage > 8).length > 0) {
      console.log('CRAZY IVAN');
      return crazyIvan(state, control);
    }
  };

  const trackLastKnownEnemy = (state, control) => {
    const [enemy] = Object.values(enemies)

    if (enemy) {
      lastKnownEnemy = enemy;
    }

    if (lastKnownEnemy) {
      const targetAngle = Math.deg.atan2(lastKnownEnemy.y - state.y, lastKnownEnemy.x - state.x);
      const gunAngle = Math.deg.normalize(targetAngle - state.angle);
      const angleDiff = Math.deg.normalize(gunAngle - state.gun.angle);
      return {command: {GUN_TURN: 0.3 * angleDiff}};
    }
  };

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
      return {command: {TURN: state.angle > 0 ? -1 : 1, ...throttleInstruction}, until: tick + 50}
    }
    if (positionInTicks.x >= autopilot.origin.x + Constants.BATTLEFIELD_WIDTH) {
      return {command: {TURN: state.angle > 0 ? 1 : -1, ...throttleInstruction}, until: tick + 50}
    }
    if (positionInTicks.y <= autopilot.origin.y) {
      return {command: {TURN: Math.abs(state.angle) < 90 ? 1 : -1, ...throttleInstruction}, until: tick + 50}
    }
    if (positionInTicks.y >= autopilot.origin.y + Constants.BATTLEFIELD_HEIGHT) {
      return {command: {TURN: Math.abs(state.angle) < 90 ? -1 : 1, ...throttleInstruction}, until: tick + 50}
    }
  };

  const moveTowardsTarget = (state, control) => {
    const enemy = whoShouldIShoot(state, control);
    const distance = enemy && Math.distance(enemy.x, enemy.y, state.x, state.y);

    if (enemy && distance > 250) {
      let targetAngle = Math.deg.atan2(enemy.y - state.y, enemy.x - state.x);
      let angleDiff = Math.deg.normalize(targetAngle - state.angle);
      return {command: {TURN: angleDiff}}
    }
  };

  const dontGetTooClose = (state, control) => {

    const tanksWithInfo = [...Object.values(enemies), ...Object.values(friendlies)]
      .filter(tank => tank.id != myId)
      .map(tank => {
        const targetAngle = Math.deg.atan2(tank.y - state.y, tank.x - state.x);
        const distance = Math.distance(tank.x, tank.y, state.x, state.y);

        return {
          tank,
          targetAngle,
          distance,
        };
      })
      .filter(info => info.distance < 150);

    sortByKey(tanksWithInfo, 'targetAngle');

    const aReallyCloseTank = tanksWithInfo.find(info => info.distance < 50);

    if (tanksWithInfo.length < 1 || !aReallyCloseTank) {
      return;
    }

    tanksWithInfo.push(tanksWithInfo[0]);

    const gaps = []

    for(let i = 0; i < tanksWithInfo.length - 1; i++) {
      let angle = tanksWithInfo[i + 1].targetAngle - tanksWithInfo[i].targetAngle;

      if (angle <= 0) {
        angle += 180
      }

      const middle = tanksWithInfo[i].targetAngle + (angle / 2);
      gaps.push({
        angle,
        middle,
        angleDiff: Math.deg.normalize(middle - state.angle)
      });
    }

    const bigGaps = gaps.filter(gap => gap.angle > 40)

    sortByKey(bigGaps, 'angleDiff');

    const closestGap = bigGaps[0];

    return {command: {TURN: closestGap.angleDiff}}
  };

  const alwaysBeDriving = (state, control) => ({command: {
    THROTTLE: 1,
  }});

  const alwaysBeShooting = (state, control) => ({command: {
    SHOOT: 0.1,
  }});

  const moveRandomly = (state, control) => ({command: {
    TURN: Math.floor(Math.random() * 20) == 2 ? Math.random()*2 - 1 : control.TURN,
  }});

  const avoidSelfCollision = (state, control) => {
    if (state.collisions.ally) {
      return [
        {command: { THROTTLE: -1, TURN: Math.random()*2 - 1}, until: tick + 50 },
        {command: { THROTTLE: 1, TURN: Math.random()*2 - 1 }, until: tick + 100}
      ]
    }
  };

  // end strategies

  tank.init(function(settings, info) {
    myId = info.id
  });

  tank.loop(function(state, control) {
    tick++;

    autopilot.update(state, control);

    [
      discoverOrigin,
      trackNearbyEnemies,
      trackNearbyFriendlies,
      alwaysBeScanning,
      alwaysBeShooting,
      counterGunTurn,
      alwaysBeDriving,
      lockRadarOnNearbyEnemies,
      moveRandomly,
      trackLastKnownEnemy,
      shootAtVisibleTanks,
      moveTowardsTarget,
      surroundEnemies,
      dontGetTooClose,
      dodgeBullets,
      avoidCollidingWithWalls,
      dontShootAllies,
      ramJamro,
      avoidSelfCollision,
    ].reduce((result, strategy) => {
      const memory = (commandMemory.get(strategy) || []).filter(record => record.until > tick);
      commandMemory.set(memory);

      if (memory[0]) {
        return Object.assign(control, memory[0].command);
      }

      const value = strategy(state, control);
      const instructions = value instanceof Array ? value : value ? [value] : [];

      if (instructions[0] && instructions[0].command) {
        Object.assign(control, instructions[0].command);
      }
      if (instructions.length > 0) {
        commandMemory.set(strategy, instructions);
      }

    }, control);

    state.radio.inbox = [];
  });


}

window.myTank = myTank
