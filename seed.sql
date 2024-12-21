-- Creating the `maps` table
CREATE TABLE IF NOT EXISTS `maps` (
  `map_id` INTEGER PRIMARY KEY,
  `map_name` TEXT,
  `created_at` TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Creating the `wormholes` table
CREATE TABLE IF NOT EXISTS `wormholes` (
  `id` INTEGER PRIMARY KEY,
  `name` TEXT,
  `created_at` TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Creating the `map_wormhole_relations` table
CREATE TABLE IF NOT EXISTS `map_wormhole_relations` (
  `map_id` INTEGER,
  `wormhole_id` INTEGER,
  `pose_x` REAL,
  `pose_y` REAL,
  `pose_z` REAL,
  `orientation_x` REAL,
  `orientation_y` REAL,
  `orientation_z` REAL,
  `orientation_w` REAL,
  `frame_id` TEXT,
  `created_at` TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  FOREIGN KEY(map_id) REFERENCES maps(map_id),
  FOREIGN KEY(wormhole_id) REFERENCES wormholes(id)
);

-- Inserting data into `maps` table
INSERT INTO `maps` (`map_id`, `map_name`) VALUES
(1, 'map_part1'),
(2, 'map_part2'),
(3, 'room1'),
(4, 'room2'),
(5, 'room3'),
(6, 'room4'),
(7, 'room5');

-- Inserting data into `wormholes` table
INSERT INTO `wormholes` (`id`, `name`) VALUES
(1, 'wh1'),
(2, 'rwh1'),
(3, 'rwh2'),
(4, 'rwh3'),
(5, 'rwh4');

-- Inserting data into `map_wormhole_relations` table
INSERT INTO `map_wormhole_relations` (`map_id`, `wormhole_id`, `pose_x`, `pose_y`, `pose_z`, `orientation_x`, `orientation_y`, `orientation_z`, `orientation_w`, `frame_id`) VALUES
(1, 1, 8.0, -5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 'map'),
(2, 1, 7.9, -5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 'map'),
(3, 2, 1.45, -3.7, 0.0, 0.0, 0.0, -0.14, 0.99, 'map'),
(4, 2, -11.5, 0.24, 0.0, 0.0, 0.0, -0.21, 0.975, 'map'),
(4, 3, -1.93, -9.46, 0.0, 0.0, 0.0, -0.81, 0.58, 'map'),
(6, 3, 8.48, -4.96, 0.0, 0.0, 0.0, -0.85, 0.52, 'map'),
(6, 4, -0.93, -4.0, 0.0, 0.0, 0.0, 0.99, 0.1, 'map'),
(5, 4, 2.2, -3.5, 0.0, 0.0, 0.0, 0.98, 0.14, 'map'),
(5, 5, -0.618, -7.87, 0.0, 0.0, 0.0, -0.8, 0.58, 'map'),
(7, 5, -6.58, -4.27, 0.0, 0.0, 0.0, -0.79, 0.611, 'map');
