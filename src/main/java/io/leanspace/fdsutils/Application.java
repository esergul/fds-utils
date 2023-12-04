package io.leanspace.fdsutils;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;
import io.leanspace.flightdynamics.orekit.OrekitLibrary;
import io.leanspace.flightdynamics.orekit.model.GroundStation;
import io.leanspace.flightdynamics.orekit.model.GroundStationVisibility;
import io.leanspace.flightdynamics.orekit.model.OrbitData;
import io.leanspace.flightdynamics.orekit.model.OrbitElements;
import io.leanspace.flightdynamics.orekit.model.OrbitElements.CoordinateType;
import io.leanspace.flightdynamics.orekit.model.OrbitElements.Tle;
import io.leanspace.flightdynamics.orekit.model.OrbitPropagationParameters;
import io.leanspace.flightdynamics.orekit.model.OrbitPropagationResult;
import io.leanspace.flightdynamics.orekit.model.PropagatorConfiguration;
import io.leanspace.flightdynamics.orekit.model.Spacecraft;
import io.leanspace.flightdynamics.orekit.orbit.propagation.NumericalOrbitPropagation;
import io.leanspace.flightdynamics.orekit.orbit.propagation.Sgp4OrbitPropagation;
import io.leanspace.fdsutils.model.GroundStationWithName;
import io.leanspace.fdsutils.model.PropagationInputs;
import lombok.SneakyThrows;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Instant;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.UUID;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Application {

  public static void main(String[] args) {
    OrekitLibrary.setup();
    runTests();
  }

  private static void runTests() {
    generateOemFile();
  }

  private static PropagationInputs getPropagationInputs() {

    final int secondsInDay = 86400;
    final int propagationDuration = 30 * secondsInDay;
    final int propagationStepInSeconds = 60;
    final String tleLine1 = "1 42829U 17042E   19338.93232017 +.00000015 +00000-0 +68468-5 0  9993";
    final String tleLine2 = "2 42829 097.5429 224.5636 0013299 211.4408 148.6018 14.91030680130155";
    final EnumSet<CoordinateType> coordinateTypesToUse = EnumSet.of(CoordinateType.CARTESIAN_ECI);
    return PropagationInputs.builder()
      .tle(new Tle(tleLine1, tleLine2))
      .startTime(Instant.now())
      .stopTime(Instant.now().plusSeconds(propagationDuration))
      .step(propagationStepInSeconds)
      .coordinateTypesToUse(coordinateTypesToUse)
      .build();
  }

  private static void generateOemFile() {

    final PropagationInputs propagationInputs = getPropagationInputs();

    final OrbitPropagationParameters inputParameters = OrbitPropagationParameters.builder()
      .spacecraft(Spacecraft.builder()
        .name("TECHNOSAT")
        .id("42829")
        .build())
      .orbitData(OrbitData.builder()
        .date(propagationInputs.getStartTime())
        .orbitElements(OrbitElements.builder()
          .type(OrbitElements.CoordinateType.TLE)
          .tle(OrbitElements.Tle.builder()
            .line1(propagationInputs.getTle().getLine1())
            .line2(propagationInputs.getTle().getLine2())
            .build())
          .build())
        .build())
      .propagatorConfiguration(PropagatorConfiguration.builder()
        .isAnalytical(true)
        .build())
      .startDate(propagationInputs.getStartTime())
      .step(propagationInputs.getStep())
      .duration(propagationInputs.getStopTime().getEpochSecond() - propagationInputs.getStartTime().getEpochSecond())
      .generateOem(true)
      .build();

    final OrbitPropagationResult output = new NumericalOrbitPropagation()
        .run(inputParameters, propagationInputs.getCoordinateTypesToUse());

    try (FileWriter writer = new FileWriter("output.oem")) {
      writer.write(output.getOemFile());
    } catch (IOException e) {
      System.err.println("Error writing to the file: " + e.getMessage());
    }
  }

  @SneakyThrows
  private static List<GroundStationWithName> getGsList() {
    final String filePath = "src/main/resources/groundStations.json";
    String jsonList = new String(Files.readAllBytes(Paths.get(filePath)));

    ObjectMapper objectMapper = new ObjectMapper();
    List<Map<String, Object>> locations = objectMapper.readValue(jsonList, new TypeReference<>() {});
    return locations.stream()
      .map(location -> GroundStationWithName.builder()
        .groundStation(GroundStation.builder()
          .id(UUID.randomUUID())
          .latitude((Double) location.get("latitude"))
          .longitude((Double) location.get("longitude"))
          .elevation(((Integer) location.get("elevation")).doubleValue())
          .build())
        .name(location.get("name").toString())
        .build())
      .collect(Collectors.toList());
  }

  @SneakyThrows
  private static List<Tle> getTleList() {
    final String filePath = "src/main/resources/tles.txt";
    final List<String> lines = Files.lines(Paths.get(filePath)).collect(Collectors.toList());
    return Stream.iterate(0, i -> i + 2)
      .limit(lines.size() / 2)
      .map(i -> new Tle(lines.get(i), (i + 1 < lines.size()) ? lines.get(i + 1) : null))
      .collect(Collectors.toList());
  }

  @SneakyThrows
  public static void generateGroundStationVisibilities() {
    List<GroundStationWithName> groundStationsWithNames = getGsList();
    final List<Tle> tles = getTleList();

    final ObjectMapper objectMapper = new ObjectMapper();
    objectMapper.registerModule(new JavaTimeModule());
    objectMapper.configure(SerializationFeature.WRITE_DATES_AS_TIMESTAMPS, false);
    objectMapper.configure(SerializationFeature.WRITE_DATE_TIMESTAMPS_AS_NANOSECONDS, false);

    List<GroundStation> groundStationsList = groundStationsWithNames.stream()
      .map(GroundStationWithName::getGroundStation)
      .collect(Collectors.toList());

    int i = 0;
    for (Tle tle : tles) {
      final PropagationInputs inputs = PropagationInputs.builder()
        .tle(tle)
        .startTime(Instant.parse("2023-12-30T16:28:42.123Z"))
        .stopTime(Instant.parse("2023-12-31T00:28:42.123Z"))
        .step(60)
        .groundStationList(groundStationsList)
        .build();
      final Sgp4OrbitPropagation sgp4 = new Sgp4OrbitPropagation(inputs.getTle());

      sgp4.setStepHandler(inputs.getStep());

      sgp4.addVisibilityDetector(inputs.getStartTime(), inputs.getStopTime(), inputs.getGroundStationList());
      sgp4.propagate(inputs.getStartTime(), inputs.getStopTime());

      final List<GroundStationVisibility> visibilities = sgp4.getVisibilities();

      // Create a list in the required format
      List<Map<String, Object>> jsonList = visibilities.stream()
        .map(visibility -> {
          Map<String, Object> jsonMap = Map.of(
            "start", visibility.getAos(),
            "end", visibility.getLos(),
            "groundstationName", Objects.requireNonNull(groundStationsWithNames.stream()
              .filter(item -> item.getGroundStation().getId().equals(visibility.getGroundStationId()))
              .findFirst()
              .map(GroundStationWithName::getName)
              .orElse(null))
          );
          return jsonMap;
        })
        .collect(Collectors.toList());

      char letter = (char) ('A' + i / 2);
      Files.write(Path.of(String.valueOf(letter) + (i%2 + 1) + " Passes.json"), objectMapper.writerWithDefaultPrettyPrinter().writeValueAsString(jsonList).getBytes());
      i++;
    }
  }

}
